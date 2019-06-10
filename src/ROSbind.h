//
// Created by safoex on 15.05.19.
//

#ifndef PROJECT_ROSBIND_H
#define PROJECT_ROSBIND_H


#include "ros/ros.h"
#include <string>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <mutex>
#include </home/safoex/Documents/abtm/src/TreeParser.h>
#include </home/safoex/Documents/abtm/src/Tree.h>



#define DEFAULT_QUEUE_SIZE 1000

namespace bt {


    class ROSbind {
    public:
        using sample = std::unordered_map<std::string, double>;
        template<class T>
        using PublisherFunction = std::function<T(sample const&)>;
        template<class T>
        using SubscriberFunction = std::function<sample(T const &)>;

        using MakeSubscriberFunction = std::function<void(ROSbind& b, YAML::Node const& node, std::string const& sub_name)>;
        using MakePublisherFunction = std::function<void(ROSbind& b, YAML::Node const& node, std::string const& pub_name)>;

        std::unordered_map<std::string, MakeSubscriberFunction> make_subscriber;
        std::unordered_map<std::string, MakePublisherFunction> make_publisher;


//    protected:
        bool online;
        std::string tests_input, tests_output;
        std::mutex lock;
        std::unordered_map<std::string, std::unordered_set<std::string>> pub_bind;
        std::unordered_map<std::string, ros::Publisher> publisher_bind;
        std::unordered_map<std::string, ros::Subscriber> subscriber_bind;
        std::unordered_map<std::string, PublisherFunction<void>> pub_by_name;
        ros::NodeHandle &n;
        Tree &tree;
        YAML::Node config;
    public:
        ROSbind(ros::NodeHandle &n, Tree &tree, std::string const &yaml_settings = "");

        void build_from_yaml_file(std::string const &yaml_settings);
        void load_publisher_from_node(std::string const& name, YAML::Node const& yaml_node);
        void load_subscriber_from_node(std::string const& name, YAML::Node const& yaml_node);
        template<class T>
        void add_subscriber(SubscriberFunction<T> subscriberFunction, std::string const &subscriber_name,
                            unsigned queue_size);

        template<class T>
        void add_publisher(PublisherFunction<T> publisherFunction,
                           std::unordered_set<std::string> used_variables, std::string const &publisher_name,
                           unsigned queue_size);

        void process(sample const &input);
        void send_output(sample const& output);

        void add_subscriber_by_type(std::string const& ros_type, YAML::Node const& node,
                std::string const& subscriber_name);

        void add_publisher_by_type(std::string const& ros_type, YAML::Node const& node,
                std::string const& publisher_name);

    };
}

#endif //PROJECT_ROSBIND_H
