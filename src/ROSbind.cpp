//
// Created by safoex on 15.05.19.
//

#include "ROSbind.h"
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/console.h>


namespace bt {


    template<class T>
    class ROSPacker {

    public:

        static unsigned get_queue_size(YAML::Node const& node) {
            if(node["queue_size"])
                return node["queue_size"].as<unsigned>();
            else
                return DEFAULT_QUEUE_SIZE;
        }
        static ROSbind::MakePublisherFunction unpacker_envelop(std::function<void(T&, ROSbind::sample const&,
                YAML::Node const&)> convert, std::function<std::vector<std::string>(YAML::Node const&)> const& get_var) {
            return [convert, get_var](ROSbind& b, YAML::Node const& node, std::string const& pub_name) -> void {
                std::vector<std::string> used_variables = get_var(node);
                bt::ROSbind::PublisherFunction<T> f = [&b, node, convert](ROSbind::sample const& out) -> T {
                    T msg;
                    convert(msg, out, node);
                    return msg;
                };
                b.add_publisher(f, {used_variables.begin(), used_variables.end()}, pub_name, ROSPacker::get_queue_size(node));
            };
        }

        static ROSbind::MakeSubscriberFunction packer_envelop(std::function<void(T const&, ROSbind::sample&,
                YAML::Node const&)> convert) {
            return [convert](ROSbind& b, YAML::Node const& node,
                             std::string const& pub_name) -> void {

                bt::ROSbind::SubscriberFunction<T> f = [&b, node, convert](T const& msg) -> ROSbind::sample {
                    ROSbind::sample in;
                    convert(msg, in, node);
                    return in;
                };
                b.add_subscriber(f, pub_name, ROSPacker::get_queue_size(node));
            };
        }

        static std::vector<std::string> get_single_var(YAML::Node const& node) {
            std::vector<std::string> vars;
            vars.push_back("");
            if(node["var"])
                vars[0] = (node["var"].as<std::string>());
            return vars;
        }

        static std::vector<std::string> get_array_var(YAML::Node const& node) {
            std::vector<std::string> vars;
            if(node["var"]) {
                for(auto v: node["vars"]) {
                    vars.push_back(v.as<std::string>());
                }
            }
            return vars;
        }

        static ROSbind::MakeSubscriberFunction single_sub();
        static ROSbind::MakeSubscriberFunction array_sub();
        static ROSbind::MakePublisherFunction single_pub();
        static ROSbind::MakePublisherFunction  array_pub();

    };

    template <class T> ROSbind::MakePublisherFunction ROSPacker<T>::single_pub() {
        return ROSPacker<T>::unpacker_envelop([](T& msg,
            ROSbind::sample const& out, YAML::Node const& node) {
        msg.data = out.at(ROSPacker::get_single_var(node)[0]);
    }, ROSPacker::get_single_var);}

    template <class T> ROSbind::MakePublisherFunction ROSPacker<T>::array_pub() {
        return ROSPacker<T>::unpacker_envelop([](T& msg,
            ROSbind::sample const& out, YAML::Node const& node) {
            auto used_variables = ROSPacker::get_array_var(node);
            for(int i = 0; i < used_variables.size(); i++)
                msg.data[i] = out.at(used_variables[i]);
    }, ROSPacker::get_array_var);}

    template <class T> ROSbind::MakeSubscriberFunction ROSPacker<T>::single_sub() {
        return ROSPacker<T>::packer_envelop([](T const& msg,
            ROSbind::sample &in, YAML::Node const& node) {
        in[ROSPacker::get_single_var(node)[0]] = msg.data;
    });}

    template <class T> ROSbind::MakeSubscriberFunction ROSPacker<T>::array_sub() {
        return ROSPacker<T>::packer_envelop([](T const& msg,
            ROSbind::sample &in, YAML::Node const& node) {
            auto used_variables = ROSPacker::get_array_var(node);
        for(int i = 0; i < used_variables.size(); i++)
            in[used_variables[i]] = msg.data[i];
    });}



//    template<class T> ROSbind::MakePublisherFunction unpacker_envelop(std::function<void(T&,
//            ROSbind::sample const&, std::vector<std::string> const&)> convert) {
//        return [convert](ROSbind& b, std::vector<std::string> const& used_variables,
//                  std::string const& pub_name, unsigned queue_size) -> ROSbind::PublisherFunction<T> {
//            bt::ROSbind::PublisherFunction<T> f = [&b, used_variables, convert](ROSbind::sample const& out) -> T {
//                T msg;
//                convert(msg, out, used_variables);
//                return msg;
//            };
//            b.add_publisher(f, {used_variables.begin(), used_variables.end()}, pub_name, queue_size);
//        };
//    }
//
//
//    template<class T> auto single_pub = unpacker_envelop<T>([](T& msg,
//            ROSbind::sample const& out, std::vector<std::string> const& used_variables) {
//        msg.data = out.at(used_variables[0]);
//    });
//
//    template<class T> auto array_pub = unpacker_envelop<T>([](T& msg,
//            ROSbind::sample const& out, std::vector<std::string> const& used_variables) {
//        for(int i = 0; i < used_variables.size(); i++)
//            msg.data[i] = out.at(used_variables[i]);
//    });
//
//    template<class T> ROSbind::MakeSubscriberFunction packer_envelop(std::function<void(T const&,
//            ROSbind::sample&, std::vector<std::string> const&)> convert) {
//        return [convert](ROSbind& b, std::vector<std::string> const& used_variables,
//                         std::string const& pub_name, unsigned queue_size) -> ROSbind::SubscriberFunction<T> {
//            std::cout << "OKAY I'M HERE " << pub_name << std::endl;
//            bt::ROSbind::SubscriberFunction<T> f = [&b, used_variables, convert](T const& msg) -> ROSbind::sample {
//                ROSbind::sample in;
//                convert(msg, in, used_variables);
//                return in;
//            };
//            b.add_subscriber(f, pub_name, queue_size);
//        };
//    }
//
//    template<class T> auto single_sub = packer_envelop<T>([](T const& msg,
//            ROSbind::sample &in, std::vector<std::string> const& used_variables) {
//        in[used_variables[0]] = msg.data;
//    });
//
//    template<class T> auto array_sub = packer_envelop<T>([](T const& msg,
//            ROSbind::sample &in, std::vector<std::string> const& used_variables) {
//        for(int i = 0; i < used_variables.size(); i++)
//            in[used_variables[i]] = msg.data[i];
//    });
//


    ROSbind::ROSbind(ros::NodeHandle &n, Tree &tree, const std::string &yaml_settings) : n(n), tree(tree) {


        make_publisher["std_msgs/UInt8"] = ROSPacker<std_msgs::UInt8>::single_pub();
        make_publisher["std_msgs/Float32"] = ROSPacker<std_msgs::Float32>::single_pub();
        make_publisher["std_msgs/Float64"] = ROSPacker<std_msgs::Float64>::single_pub();

        make_publisher["std_msgs/Float32MultiArray"] = ROSPacker<std_msgs::Float32MultiArray>::array_pub();
        make_publisher["std_msgs/Float32MultiArray"] = ROSPacker<std_msgs::Float64MultiArray>::array_pub();

        make_publisher["geometry_msgs/Point"] = ROSPacker<geometry_msgs::Point>::unpacker_envelop([](geometry_msgs::Point& msg,
                ROSbind::sample const& out, YAML::Node const& node) {
            msg.x = out.at(node["var"]["x"].as<std::string>());
            msg.y = out.at(node["var"]["y"].as<std::string>());
            msg.z = out.at(node["var"]["z"].as<std::string>());
        }, [](YAML::Node const& node) -> std::vector<std::string> {
           return {
               node["var"]["x"].as<std::string>(),
               node["var"]["y"].as<std::string>(),
               node["var"]["z"].as<std::string>()
           };
        });

        make_publisher["geometry_msgs/PointStamped"] = ROSPacker<geometry_msgs::PointStamped>::unpacker_envelop([](geometry_msgs::PointStamped& msg,
                ROSbind::sample const& out, YAML::Node const& node) {
            msg.point.x = out.at(node["var"]["point"]["x"].as<std::string>());
            msg.point.y = out.at(node["var"]["point"]["y"].as<std::string>());
            msg.point.z = out.at(node["var"]["point"]["z"].as<std::string>());
            msg.header.frame_id = node["var"]["header"]["frame_id"].as<std::string>();
            msg.header.seq = static_cast<unsigned int>(out.at(node["var"]["header"]["seq"].as<std::string>()));
            msg.header.stamp.fromSec(out.at(node["var"]["header"]["stamp"].as<std::string>()));
        }, [](YAML::Node const& node) -> std::vector<std::string> {
            return {
                    node["var"]["point"]["x"].as<std::string>(),
                    node["var"]["point"]["y"].as<std::string>(),
                    node["var"]["point"]["z"].as<std::string>(),
                    node["var"]["header"]["seq"].as<std::string>(),
                    node["var"]["header"]["stamp"].as<std::string>()
            };
        });

        make_subscriber["std_msgs/UInt8"] = ROSPacker<std_msgs::UInt8>::single_sub();
        make_subscriber["std_msgs/Float32"] = ROSPacker<std_msgs::Float32>::single_sub();
        make_subscriber["std_msgs/Float64"] = ROSPacker<std_msgs::Float64>::single_sub();

        make_subscriber["std_msgs/Float32MultiArray"] = ROSPacker<std_msgs::Float32MultiArray>::array_sub();
        make_subscriber["std_msgs/Float64MultiArray"] = ROSPacker<std_msgs::Float64MultiArray>::array_sub();

        make_subscriber["geometry_msgs/Point"] = ROSPacker<geometry_msgs::Point>::packer_envelop([](geometry_msgs::Point const& msg,
                ROSbind::sample & in, YAML::Node const& node) {
            in[node["var"]["x"].as<std::string>()] = msg.x;
            in[node["var"]["y"].as<std::string>()] = msg.y;
            in[node["var"]["z"].as<std::string>()] = msg.z;
        });

        make_subscriber["geometry_msgs/PointStamped"] = ROSPacker<geometry_msgs::PointStamped>::packer_envelop([](geometry_msgs::PointStamped const& msg,
                ROSbind::sample & in, YAML::Node const& node) {
            in[node["var"]["point"]["x"].as<std::string>()] = msg.point.x;
            in[node["var"]["point"]["y"].as<std::string>()] = msg.point.y;
            in[node["var"]["point"]["z"].as<std::string>()] = msg.point.z;
            in[node["var"]["header"]["seq"].as<std::string>()] = msg.header.seq;
            in[node["var"]["header"]["stamp"].as<std::string>()] = msg.header.stamp.toSec();
        });


        online = false;
        if(!yaml_settings.empty()) {
            build_from_yaml_file(yaml_settings);
        }


    }

    template<class T>
    void ROSbind::add_publisher(ROSbind::PublisherFunction<T> publisherFunction,
                                std::unordered_set<std::string> used_variables,
                                std::string const &publisher_name, unsigned queue_size) {
        auto const &pf = publisherFunction;
        publisher_bind[publisher_name] = n.advertise<T>(publisher_name, queue_size);
        auto const& p = publisher_bind[publisher_name];
        PublisherFunction<void> wrap_pub_function = [this, &p, pf, used_variables](sample const& s)
                -> void {
            sample changes;
            for(auto const& uv: used_variables) {
                changes[uv] = tree[uv];
            }
            p.publish(pf(changes));
        };
        pub_by_name[publisher_name] = (wrap_pub_function);
        for (auto const &uv: used_variables)
            pub_bind[uv].insert(publisher_name);
    }

    template<class T>
    void ROSbind::add_subscriber(ROSbind::SubscriberFunction<T> subscriberFunction, std::string const &subscriber_name,
                                 unsigned queue_size) {
        auto const &sf = subscriberFunction;
        auto wrap_sub_function = [this, sf](T input_raw)
                -> void {
            process(sf(input_raw));
        };
        boost::function<void(T)> wboost(wrap_sub_function);
        subscriber_bind[subscriber_name] = n.subscribe<T>(subscriber_name, queue_size, wboost);
    }

    void ROSbind::process(const bt::ROSbind::sample &input) {
        std::lock_guard<std::mutex> guard(lock);
        if(online) {
            ROS_DEBUG_STREAM("BT " << this->tree.get_root_name() << " on input");
            ROS_DEBUG_STREAM(print_sample(input));
        }
        auto output = tree.callback(input);
        if(output.size()) {
            std::cout << "----OUTPUT----" << std::endl;
            for(auto const& ov: output) {
                std::cout <<ov.first << ' ' << ov.second << std::endl;
            }

            std::cout << "--------------" << std::endl;
            send_output(output);
        }
        if(online) {
            ROS_DEBUG_STREAM("BT " << this->tree.get_root_name() << "has output of " << output.size());
            if (!output.empty())
                ROS_DEBUG_STREAM(print_sample(output));
        }
    }

    void ROSbind::send_output(const bt::ROSbind::sample &output) {
        std::unordered_set<std::string> pubs_to_send;
        for(auto const& ov: output) {
            pubs_to_send.insert(pub_bind[ov.first].begin(),pub_bind[ov.first].end());
        }
        for(auto const& ps: pubs_to_send) {
            auto const& p = pub_by_name[ps];
            p(output);
        }
    }

    void ROSbind::add_publisher_by_type(std::string const &ros_type, YAML::Node const& node,
                                        std::string const &publisher_name) {
        if(make_publisher.count(ros_type))
            make_publisher[ros_type](*this, node, publisher_name);

    }

    void ROSbind::add_subscriber_by_type(std::string const &ros_type, YAML::Node const& node,
            std::string const &subscriber_name) {
        if(make_subscriber.count(ros_type)) {
            std::cout << ros_type << std::endl;
            make_subscriber[ros_type](*this, node, subscriber_name);
        }
    }

    void ROSbind::load_publisher_from_node(std::string const& name, YAML::Node const &yaml_node) {
        try {
            add_publisher_by_type(yaml_node["type"].as<std::string>(), yaml_node, name);
        }
        catch (YAML::Exception & e) {
            LOG_DEBUG(e.what());
            throw YAML::Exception(e);
        }
    }

    void ROSbind::load_subscriber_from_node(std::string const& name, YAML::Node const &yaml_node) {
        try {
            add_subscriber_by_type(yaml_node["type"].as<std::string>(), yaml_node, name);
        }
        catch (YAML::Exception & e) {
            LOG_DEBUG(e.what());
            throw YAML::Exception(e);
        }
    }

    void ROSbind::build_from_yaml_file(std::string const &yaml_settings) {
        try {
            config = YAML::LoadFile(yaml_settings);
        }
        catch(YAML::Exception &e) {
            throw YAML::Exception(YAML::Mark::null_mark(), "smth wrong with bind description file " + (std::string)e.what());
        }
        const std::string ROSBIND_NAME = "ROSbind",
        PUBLISHERS_NAME = "publishers",
        SUBSCRIBERS_NAME = "subscribers";
        try {
            if (config["common"]["parameters"]["online"].as<int>()) {
                online = true;
            } else {
                try {
                    tests_input  = config["common"]["input"].as<std::string>();
                    tests_output = config["common"]["output"].as<std::string>();
                }
                catch (YAML::Exception &e) {
                    throw YAML::Exception(YAML::Mark::null_mark(),
                                          "smth wrong with tests filename " + (std::string) e.what());
                }
            }
        }
        catch (YAML::Exception &e) {
            throw YAML::Exception(YAML::Mark::null_mark(),
                                  "smth wrong with online parameter " + (std::string) e.what());
        }

        if(config[ROSBIND_NAME]) {
            if(config[ROSBIND_NAME][PUBLISHERS_NAME]) {
                for(auto const& yaml_node: config[ROSBIND_NAME][PUBLISHERS_NAME]) {
                    load_publisher_from_node(yaml_node.first.as<std::string>(), yaml_node.second);
                }
            }
            if(config[ROSBIND_NAME][SUBSCRIBERS_NAME]) {
                for(auto const& yaml_node: config[ROSBIND_NAME][SUBSCRIBERS_NAME]) {
                    load_subscriber_from_node(yaml_node.first.as<std::string>(), yaml_node.second);
                }
            }
        }
    }
}