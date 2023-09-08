#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include "std_msgs/String.h"

static std::string interrupt_event;

// 인터럽트 이벤트 콜백 함수
void interruptCallback(const std_msgs::String::ConstPtr& msg)
{
    printf("인터럽트 콜백 함수 - 감지된 이벤트: %s\n", msg->data.c_str());
    interrupt_event = msg->data;
}

class InterruptEvent : public BT::SyncActionNode
{
public:
    InterruptEvent(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        // 'interrupt_event' 토픽을 구독하는 ROS 노드 핸들 및 구독자 설정
        sub_ = node_.subscribe("interrupt_event", 1000, interruptCallback);
    }

    // 제공되는 포트 목록을 정의하는 정적 메서드
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("event") };
    }

    virtual BT::NodeStatus tick() override
    {
        std::string expect_event;

        // "event" 입력 포트에서 이벤트 문자열을 가져오기 시도
        if (!getInput<std::string>("event", expect_event)) {
            throw BT::RuntimeError("필수 입력 [event]이 누락되었습니다.");
        }

        interrupt_event = "";
        ros::spinOnce();  // ROS 이벤트 처리

        // 감지된 이벤트가 예상 이벤트와 같으면 실패 반환, 그렇지 않으면 성공 반환
        if (interrupt_event == expect_event)
        {
            printf("감지된 이벤트: %s\n", interrupt_event.c_str());
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            // printf("감지안된 이벤트: %s\n", interrupt_event.c_str());
            return BT::NodeStatus::SUCCESS;
        }
    }

private:
    ros::NodeHandle node_;
    ros::Subscriber sub_;
};
