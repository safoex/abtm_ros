#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ROSbind.cpp"

#include <fstream>
#include <sstream>
#include <cstdlib>

void activation_hack(ros::NodeHandle &n) {
    auto const& test_publisher = n.advertise<std_msgs::Float32>("counter2", 10);
    ros::Rate sleep_1s(0.5);
    sleep_1s.sleep();


    std_msgs::Float32 msg_float;
    msg_float.data = 3;
    test_publisher.publish(msg_float);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    bt::Tree t;
    std::string tree_yaml = "tree.yaml";
    bt::TreeParser tp(&t, "tree.yaml");

    tp.load();

    bt::ROSbind rb(n, t, tree_yaml);

    activation_hack(n);

    std::ofstream("dot.txt") << t.dot_tree_description();
    system("dot -Tpdf dot.txt > tree.pdf");

    rb.send_output(t.start());
    std::ofstream("states.txt") << t.dot_tree_description(true);


    if(rb.online) {
        ros::Rate sleep_small(20);
        std::string dot_description;
        while (ros::ok()) {
            sleep_small.sleep();
            ros::spinOnce();

            std::string new_desc = t.dot_tree_description(true);
            if (new_desc != dot_description) {
                std::ofstream("states.txt") << new_desc;
                system("dot -Tpdf states.txt > states.pdf");
                dot_description = new_desc;
            }
        }
    }
    else {
        std::ofstream(rb.tests_output) << tp.apply_samples(rb.tests_input);
    }
    return 0;
}
