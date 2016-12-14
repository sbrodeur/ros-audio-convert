#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/base/gstbasesrc.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <unistd.h>

#include "audio_common_msgs/AudioData.h"

namespace audio_transport
{
  class RosGstConvert
  {
    public:
      RosGstConvert(const std::string& location)
      {
        GstPad *audiopad;

        _loop = g_main_loop_new(NULL, false);

        _pipeline = gst_pipeline_new("app_pipeline");
        
        _source = gst_element_factory_make("appsrc", "app_source");
        g_object_set (G_OBJECT(_source), "stream-type", 0, "format", GST_FORMAT_TIME, NULL);
        g_object_set( G_OBJECT(_source), "block", TRUE, NULL);
        gst_bin_add( GST_BIN(_pipeline), _source);
        
        _queue = gst_element_factory_make("queue", NULL);
        g_object_set( G_OBJECT(_queue), "max-size-buffers", 0, NULL);
        g_object_set( G_OBJECT(_queue), "max-size-time", 0, NULL);
        g_object_set( G_OBJECT(_queue), "max-size-bytes", 0, NULL);
        g_object_set( G_OBJECT(_queue), "silent", TRUE, NULL);
        
        gst_bin_add( GST_BIN(_pipeline), _queue);
        if (gst_element_link(_source, _queue) != TRUE)
		{
			std::cout << "Error occured in linking audio queue \n";
			exit(1);
		}

        _decoder = gst_element_factory_make("decodebin", "decoder");
		g_signal_connect(_decoder, "pad-added", G_CALLBACK(cb_newpad),this);
		gst_bin_add( GST_BIN(_pipeline), _decoder);
		if (gst_element_link(_queue, _decoder) != TRUE)
		{
			std::cout << "Error occured in linking decoder \n";
			exit(1);
		}
		          
		_audio = gst_bin_new("audiobin");
		_convert = gst_element_factory_make("audioconvert", "convert");
		_encoder = gst_element_factory_make("wavenc", "encoder");
		audiopad = gst_element_get_static_pad(_convert, "sink");
		
		_sink = gst_element_factory_make("filesink", "sink");
		g_object_set( G_OBJECT(_sink), "location", location.c_str(), NULL);
		
		gst_bin_add_many( GST_BIN(_audio), _convert, _encoder, _sink, NULL);
		if (gst_element_link_many (_convert, _encoder, _sink, NULL) != TRUE)
		{
			std::cout << "Error occured in linking audio converter, encoder and sink \n";
			exit(1);
		}
		
		gst_element_add_pad(_audio, gst_ghost_pad_new("sink", audiopad));
		gst_object_unref(audiopad);

		gst_bin_add(GST_BIN(_pipeline), _audio);
		
        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );
      }

      void close()
      {
    	  /* cleanup */
    	  gst_element_set_state (_pipeline, GST_STATE_NULL);
    	  gst_object_unref (GST_OBJECT (_pipeline));
      }
      
      void onAudio(const audio_common_msgs::AudioDataConstPtr &msg)
      {
        GstBuffer *buffer = gst_buffer_new_and_alloc(msg->data.size());
        gst_buffer_fill(buffer, 0, &msg->data[0], msg->data.size());
        GstFlowReturn ret;

        ret = gst_app_src_push_buffer(GST_APP_SRC_CAST (_source), buffer);
        if (ret != GST_FLOW_OK)
        {
        	std::cout << "Error occured in enqueing data into source \n";
        }
        //g_signal_emit_by_name(_source, "push-buffer", buffer, &ret);
      }

    private:
      
      static void cb_newpad (GstElement *decodebin, GstPad *pad, 
                                   gpointer data)
		{
    	  RosGstConvert *client = reinterpret_cast<RosGstConvert*>(data);
		
		  GstCaps *caps;
		  GstStructure *str;
		  GstPad *audiopad;
		
		  /* only link once */
		  audiopad = gst_element_get_static_pad (client->_audio, "sink");
		  if (GST_PAD_IS_LINKED (audiopad)) 
		  {
			g_object_unref (audiopad);
			return;
		  }
		
		  /* check media type */
		  caps = gst_pad_query_caps (pad, NULL);
		  str = gst_caps_get_structure (caps, 0);
		  if (!g_strrstr (gst_structure_get_name (str), "audio")) {
			gst_caps_unref (caps);
			gst_object_unref (audiopad);
			
			std::cout << "Error occured in adding new pad for decoder \n";
			return;
		  }
		
		  gst_caps_unref (caps);
		
		  /* link'n'play */
		  gst_pad_link (pad, audiopad);
		
		  g_object_unref (audiopad);
		}
      
      
      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_sink, *_queue, *_decoder, *_convert, *_encoder, *_audio;
      GstElement *_playbin;
      GMainLoop *_loop;

      bool _paused;
  };
}

int main(int argc, char **argv){

	ros::Time::init();
	gst_init(&argc, &argv);

	std::string input_rosbag = "input.bag";
	std::string output_wav = "output.wav";
	std::string input_audio_topic = "/audio";

	namespace po = boost::program_options;
	po::options_description desc("Allowed options");
	desc.add_options()
	("help,h", "describe arguments")
	("output,o", po::value(&output_wav), "set output WAV file")
	("input,i", po::value(&input_rosbag), "set input rosbag file")
	("input-audio-topic,t", po::value(&input_audio_topic), "set topic of the input audio_common_msgs/AudioData messages");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		std::cout << desc << "\n";
		return 1;
	}

	rosbag::Bag input(input_rosbag, rosbag::bagmode::Read);
	audio_transport::RosGstConvert client(output_wav);

	std::vector<std::string> topics;
	topics.push_back(input_audio_topic);
	rosbag::View view(input, rosbag::TopicQuery(topics));
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		audio_common_msgs::AudioDataConstPtr msg = m.instantiate<audio_common_msgs::AudioData>();
		if (msg != NULL){
			// Send data to client
			client.onAudio(msg);
			
			// FIXME: a small delay is needed here. Maybe for the client thread to have time to queue the data?
			usleep(1000);
		}
	}

	input.close();
	client.close();
	
	return 0;
}

