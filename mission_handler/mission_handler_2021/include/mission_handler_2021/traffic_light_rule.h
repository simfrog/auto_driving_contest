#pragma once
#include <traffic_light_msgs/Light.h>
#include <deque>
#include <algorithm>
enum class TrafficLightState {STRANGE, STRAIGHT, LEFT, YELLOW, RED, STRAIGHT_AND_LEFT};

struct TrafficLightRule {
    constexpr static int LEN_QUEUE = 10;
    constexpr static int MINIMUM_SCORE = LEN_QUEUE / 2;
    TrafficLightState getPromisingState(traffic_light_msgs::Light l) {
        if ( 1 > l.light || l.light > 5) throw std::runtime_error("only straight, left, yellow, red, straight_and_left are allowed");

        //store light queue
        recent_light_queue.push_back(l);
        if (recent_light_queue.size() > LEN_QUEUE) recent_light_queue.pop_front();

        //find maximum score light with score
        std::vector<int> score_vec;
        score_vec.resize(5);
        for(auto& l : recent_light_queue)
            score_vec[l.light - 1]++;
        auto it_max = std::max_element(score_vec.begin(), score_vec.end());
        if (*it_max < MINIMUM_SCORE) return TrafficLightState::STRANGE;

        int idx = it_max - score_vec.begin();
        if (idx == 0) {
            //ROS_WARN("Straight!");
            return TrafficLightState::STRAIGHT;
        }
        else if (idx == 1) {
            //ROS_WARN("Left!");
            return TrafficLightState::LEFT;
        }
        else if (idx == 2) {
            //ROS_WARN("Yellow!");
            return TrafficLightState::YELLOW;
        }
        else if (idx == 3) {
            //ROS_WARN("Red!");
            return TrafficLightState::RED;
        }
        else if (idx == 4) {
            //ROS_WARN("Straight and Left!");
            return TrafficLightState::STRAIGHT_AND_LEFT;
        }
        else throw std::runtime_error("impossible situation happened in TrafficLightRule");
    }
    std::deque<traffic_light_msgs::Light> recent_light_queue;        
};