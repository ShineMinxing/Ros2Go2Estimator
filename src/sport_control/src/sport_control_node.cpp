#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <iostream>  // 引入 cout
#include <cstdlib>   // 引入 system
#include <chrono>    // 引入 chrono，用于计算时间差
#include <iomanip>   // 添加：用于 std::setprecision
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>

class JoystickControlNode : public rclcpp::Node
{
public:
    JoystickControlNode() : Node("sport_control_node")
    {
        // 通过参数获取网络接口名称，设置默认值为 "enxc8a3627ff10b" 或其他有效接口
        this->declare_parameter<std::string>("network_interface", "enxc8a3627ff10b");
        std::string network_interface = this->get_parameter("network_interface").as_string();

        // 初始化Unitree通道工厂
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

        // 初始化句子
        Last_Operation = "Joystick Control Init";

        // 创建订阅者，订阅/joy话题
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoystickControlNode::joy_callback, this, std::placeholders::_1));

        // 创建订阅者，订阅/SportCmd话题
        sport_cmd_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "SMXFE/SportCmd", 10, std::bind(&JoystickControlNode::sport_cmd_callback, this, std::placeholders::_1));
        
        // 用于发布语音对话控制命令到 "voice_chat/control" 话题
        sport_cmd_pub = this->create_publisher<std_msgs::msg::String>("SMXFE/ModeCmd", 10);


        // 记录初始化时的时间
        Last_Operation_Time = this->get_clock()->now();

        sport_client = std::make_unique<unitree::robot::go2::SportClient>();
        sport_client->SetTimeout(10.0f);
        sport_client->Init();

        motion_client = std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
        motion_client->SetTimeout(10.0f); 
        motion_client->Init();

        Vui_client = std::make_unique<unitree::robot::go2::VuiClient>();
        Vui_client->SetTimeout(1.0f); 
        Vui_client->Init();

        RCLCPP_INFO(this->get_logger(), "JoystickControlNode 已启动");
    }

private:

    std::unique_ptr<unitree::robot::go2::SportClient> sport_client;
    std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> motion_client;
    std::unique_ptr<unitree::robot::go2::VuiClient> Vui_client;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sport_cmd_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sport_cmd_pub;

    // 获取按钮输入状态
    int Buttons[8] = {0};
    float Axes[8] = {0};

    int32_t ErrorCode = 0;

    bool JoystickEnable = 0;
    bool AIModeEnable = 0;
    bool WalkUprightEnable = 0;
    bool FreeJumpEnable = 0;
    bool FreeAvoidEnable = 0;
    bool FreeBoundEnable = 0;
    bool ForwardClimbingEnable = 0;
    bool ContinuousGaitEnable = 0;
    bool ChattingEnable = 0;
    float MotionEnable = 0;
    float SpeedScalse = 0.25;

    std::string Last_Operation;
    rclcpp::Time Last_Operation_Time;
    float Last_Operation_Duration_Time;

    // 发布语音对话命令的函数
    void publishVoiceChatCommand(const std::string &cmd)
    {
        std_msgs::msg::String msg;
        msg.data = cmd;
        sport_cmd_pub->publish(msg);
        std::cout << "发布语音命令: " << cmd << std::endl;
    }

    // 回调函数，处理joy消息
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // 清屏
        std::system("clear");

        obtain_key_value(msg);

        joystick_state_print();

        execute_action();
    }

    // 回调函数，处理SportCmd消息
    void sport_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 5)
        {
            int action = static_cast<int>(msg->data[0]);  // Convert double to int
            Actions(action, msg->data[1], msg->data[2], msg->data[3], msg->data[4]);
        }
    }

    void obtain_key_value(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        Buttons[0] = msg->buttons[0]; // A
        Buttons[1] = msg->buttons[1]; // B
        Buttons[2] = msg->buttons[2]; // X
        Buttons[3] = msg->buttons[3]; // Y
        Buttons[4] = msg->buttons[4]; // LB
        Buttons[5] = msg->buttons[5]; // RB
        Buttons[6] = msg->buttons[6]; // Back
        Buttons[7] = msg->buttons[7]; // Start

        Axes[0] = msg->axes[0]; // 左摇杆 Left 1; Right -1;
        Axes[1] = msg->axes[1]; // 左摇杆 Up 1; Down -1;
        Axes[2] = msg->axes[2]; // 左扳机 1 -> -1
        Axes[3] = msg->axes[3]; // 右摇杆 Left 1 -> -1 Right;
        Axes[4] = msg->axes[4]; // 右摇杆 Up 1 -> -1 Down;
        Axes[5] = msg->axes[5]; // 右扳机 1 -> -1
        Axes[6] = msg->axes[6]; // 左方向键 Left 1; Right -1;
        Axes[7] = msg->axes[7]; // 左方向键 Up 1; Down -1;
    }

    void joystick_state_print()
    {
        // 打印时间差及上次动作
        rclcpp::Duration elapsed_time = this->get_clock()->now() - Last_Operation_Time;
        Last_Operation_Duration_Time = elapsed_time.seconds();
        std::cout << "Error Code: " << ErrorCode << std::endl;
        std::cout << "Last Operation: " << Last_Operation << "   " << Last_Operation_Duration_Time << "s passed." << std::endl;
        
        if(Axes[2]<-0.9 && Axes[5]<-0.9 && Last_Operation_Duration_Time > 0.5)
        {
            JoystickEnable = 1 - JoystickEnable;
            Last_Operation_Time = this->get_clock()->now();  // 更新存储时间

            if(JoystickEnable)
                Last_Operation = "Unlock the joystick.";
            else
                Last_Operation = "Lock the joystick.";

            if(AIModeEnable)
            {
                AIModeEnable = 0;
                if(WalkUprightEnable)
                    ErrorCode = sport_client->WalkUpright(0);
                WalkUprightEnable = 0;
                if(FreeJumpEnable)
                    ErrorCode = sport_client->FreeJump(0);
                FreeJumpEnable = 0;
                if(FreeAvoidEnable)
                    ErrorCode = sport_client->FreeBound(0);
                FreeAvoidEnable = 0;
                if(FreeBoundEnable)
                    ErrorCode = sport_client->FreeAvoid(0);
                FreeBoundEnable = 0;
                if(ContinuousGaitEnable)
                    ErrorCode = sport_client->ContinuousGait(0);
                ContinuousGaitEnable = 0;
                ErrorCode = motion_client->SelectMode("normal");
                ChattingEnable = 0;
                publishVoiceChatCommand("stop");
            }
        }

        if(!JoystickEnable)
            std::cout << "Press Left Trigger and Right Trigger to Unlock " << std::endl;
        else
            std::cout << "Press Left Trigger and Right Trigger to Lock " << std::endl;

        std::cout << "Pressed Button " <<": ";

        // 打印当前按下的按钮
        bool button_pressed = false;
        for (int i = 0; i < 8; ++i)
        {
            if (Buttons[i] == 1)  // 判断该按钮是否按下
            {
                std::cout << "Button " << i << " --";
                button_pressed = true;
            }
        }
        if (!button_pressed)
        {
            std::cout << "No Button;";
        }
        std::cout << std::endl;

        // 打印摇杆数据，使用 std::cout
        std::cout << std::fixed << std::setprecision(1); 
        for (int i = 0; i < 8; ++i)
        {
            std::cout << i << ": " << Axes[i] <<"; ";
        }
        std::cout << std::endl;
    }

    // 执行与按键对应的操作
    void execute_action()
    {
        if(Buttons[6]) // 急停
        {
            Actions(16000000,0,0,0,0);
        }

        if(Buttons[4] && Buttons[5] && JoystickEnable && Last_Operation_Duration_Time > 0.5) // 按住LB RB， 开启或关闭AI模式
        {

            Actions(14150000,0,0,0,0);
        }

        if(Axes[5] < -0.5 && Axes[2] > 0.9 && JoystickEnable) // 只按住RT键，进行运动判断
        {
            Actions(25202123,Axes[0],Axes[1],Axes[3],Axes[4]);
        }

        if(Axes[5] < -0.5 && Axes[2] > 0.9 && JoystickEnable && !AIModeEnable && Last_Operation_Duration_Time > 0.5) // 常规模式，只按住RT键，进行动作判断
        {
            Actions(25161700,Axes[6],Axes[7], 0, 0);
            if(Buttons[0])
            {
                Actions(25100000,0,0,0,0);
            }
            else if(Buttons[1])
            {
                Actions(25110000,0,0,0,0);
            }
            else if(Buttons[2])
            {
                Actions(25120000,0,0,0,0);
            }
            else if(Buttons[3])
            {
                Actions(25130000,0,0,0,0);
            }
            else if(Buttons[7])
            {
                Actions(25170000,0,0,0,0);
            }
        }
        else if(Axes[2] < -0.5 && Axes[5] > 0.9 && JoystickEnable && !AIModeEnable && Last_Operation_Duration_Time > 0.5) // 常规模式，只按住LT键
        {
            Actions(20212324,Axes[0],Axes[1],Axes[3],Axes[4]);
            if(Buttons[0])
            {
                Actions(22100000,0,0,0,0);
            }
            else if(Buttons[1])
            {
                Actions(22110000,0,0,0,0);
            }
            else if(Buttons[2])
            {
                Actions(22120000,0,0,0,0);
            }
            else if(Buttons[3])
            {
                Actions(22130000,0,0,0,0);
            }
            else if(Buttons[7])
            {
                Actions(22170000,0,0,0,0);
            }
        }
        else if(Axes[5] < -0.5 && Axes[2] > 0.9 && JoystickEnable && AIModeEnable && Last_Operation_Duration_Time > 0.5) // AI模式，只按住RT键，进行动作判断
        {
            if(Buttons[0])
            {
                Actions(125100000,0,0,0,0);
            }
            else if(Buttons[1])
            {
                Actions(125110000,0,0,0,0);
            }
            else if(Buttons[2])
            {
                Actions(125120000,0,0,0,0);
            }
            else if(Buttons[3])
            {
                Actions(125130000,0,0,0,0);
            }
            Actions(125262700,Axes[6],Axes[7],0,0);
        }
    }

    void Actions(int Action, double Value1, double Value2, double Value3, double Value4)
    {
        // Action命名规则： 00 00 00 00 00，前两位表示模式，后面四个两位，表示同时4个按键按下
        // 按键的两位AB，A 1/2表示按键/摇杆，B 0～7表示不同的按键名/摇杆名
        switch (Action) {
            case 16000000:
                JoystickEnable = 0;
                Last_Operation = "Pause. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->Damp();
                break;
            case 14150000:
                AIModeEnable = 1 - AIModeEnable;
                if(AIModeEnable)
                    Last_Operation = "AI Mode Start. ";
                else
                    Last_Operation = "AI Mode Stop. ";
                SpeedScalse = 0.1;
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = motion_client->SelectMode("ai");
                break;
            case 25202123:
                if(abs(Value2)>0.1 || abs(Value1)>0.1 || abs(Value3)>0.1)
                {
                    float Leftward_Speed = 1 * SpeedScalse * Value1;
                    float Forward_Speed = 2.5 * SpeedScalse * Value2;
                    float Turning_Speed = 4 * SpeedScalse * Value3;
        
                    if(Forward_Speed>=0)
                        Forward_Speed = Forward_Speed / 2.5 * 3.8;
        
                    Last_Operation = "Moving to Forward: " + std::to_string(Forward_Speed) 
                    + "; Leftward: " + std::to_string(Leftward_Speed)
                    + "; Turning: " + std::to_string(Turning_Speed);
        
                    Last_Operation_Time = this->get_clock()->now();
        
                    std::cout << "Moving... " << std::endl;
        
                    ErrorCode = sport_client->Move(Forward_Speed, Leftward_Speed, Turning_Speed);
                }
                break;
            case 25161700:
                if(Value2==-1)
                {
                    Last_Operation = "Speed Scale 25%. ";
                    Last_Operation_Time = this->get_clock()->now();
                    SpeedScalse = 0.25;
                }
                else if(Value1==1)
                {
                    Last_Operation = "Speed Scale 50%. ";
                    Last_Operation_Time = this->get_clock()->now();
                    SpeedScalse = 0.5;
                }
                else if(Value1==-1)
                {
                    Last_Operation = "Speed Scale 75%. ";
                    Last_Operation_Time = this->get_clock()->now();
                    SpeedScalse = 0.75;
                }
                else if(Value2==1)
                {
                    Last_Operation = "Speed Scale 100%. ";
                    Last_Operation_Time = this->get_clock()->now();
                    SpeedScalse = 1.0;
                }
                break;
            case 25100000:
                Last_Operation = "Stand up. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->RecoveryStand();
                break;
            case 25110000:
                Last_Operation = "Sit Down. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->StandDown();
                break;
            case 25120000:
                ForwardClimbingEnable = 1 - ForwardClimbingEnable;
                if(ForwardClimbingEnable)
                {
                    ErrorCode = sport_client->SwitchGait(3);
                    Last_Operation = "Forward Climbing Pattern Start. ";
                }
                else
                {
                    ErrorCode = sport_client->SwitchGait(3);
                    Last_Operation = "Trot Running Pattern Start. ";
                }
                Last_Operation_Time = this->get_clock()->now();
                break;
            case 25130000:
                ContinuousGaitEnable = 1 - ContinuousGaitEnable;
                if(ContinuousGaitEnable)
                    Last_Operation = "Continuous Gait Start. ";
                else
                    Last_Operation = "Continuous Gait Stop. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->ContinuousGait(ContinuousGaitEnable);
                break;
            case 25170000:
                Last_Operation = "Get Ready to Move. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->BalanceStand();
                break;
            case 125100000:
                WalkUprightEnable = 1 - WalkUprightEnable;
                if(WalkUprightEnable)
                    Last_Operation = "WalkUpright Start. ";
                else
                    Last_Operation = "WalkUpright Stop. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->WalkUpright(WalkUprightEnable);
                break;
            case 125110000:
                FreeJumpEnable = 1 - FreeJumpEnable;
                if(FreeJumpEnable)
                    Last_Operation = "FreeJump Start. ";
                else
                    Last_Operation = "FreeJump Stop. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->FreeJump(FreeJumpEnable);
                break;
            case 125120000:
                FreeBoundEnable = 1 - FreeBoundEnable;
                if(FreeBoundEnable)
                    Last_Operation = "FreeBound Start. ";
                else
                    Last_Operation = "FreeBound Stop. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->FreeBound(FreeBoundEnable);
                break;
            case 125130000:
                FreeAvoidEnable = 1 - FreeAvoidEnable;
                if(FreeAvoidEnable)
                    Last_Operation = "FreeAvoid Start. ";
                else
                    Last_Operation = "FreeAvoid Stop. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->FreeAvoid(FreeAvoidEnable);
                break;
            case 125262700:
                if(Value2==-1)
                {
                    Last_Operation = "BackFlip. ";
                    Last_Operation_Time = this->get_clock()->now();
                    ErrorCode = sport_client->BackFlip();
                }
                else if(Value1==1)
                {
                    Last_Operation = "LeftFlip. ";
                    Last_Operation_Time = this->get_clock()->now();
                    ErrorCode = sport_client->LeftFlip();
                }
                else if(Value1==-1)
                {
                    Last_Operation = "LeftFlip. ";
                    Last_Operation_Time = this->get_clock()->now();
                    ErrorCode = sport_client->LeftFlip();
                }
                else if(Value2==1)
                {
                    Last_Operation = "FrontFlip. ";
                    Last_Operation_Time = this->get_clock()->now();
                    ErrorCode = sport_client->FrontFlip();
                }
                break;
            
            case 20212324:
                if(abs(Value1)>0.1 || abs(Value2)>0.1 || abs(Value3)>0.1 || abs(Value4)>0.1)
                {
                    float RollAngle = -0.75 * SpeedScalse * Value1;
                    float PitchAngle = 0.75 * SpeedScalse * Value2;
                    float YawAngle = 0.6 * SpeedScalse * Value3;
                    float Height = 0.03 * SpeedScalse * Value4;
        
                    if(Height<=0)
                        Height = Height / 0.03 * 0.18;
        
                    Last_Operation = "RollAngle: " + std::to_string(RollAngle) 
                    + "; PitchAngle: " + std::to_string(PitchAngle)
                    + "; YawAngle: " + std::to_string(YawAngle)
                    + "; Height: " + std::to_string(Height);
        
                    Last_Operation_Time = this->get_clock()->now();
        
                    std::cout << "Twisting... " << std::endl;
        
                    ErrorCode = sport_client->Euler(RollAngle, PitchAngle, YawAngle);
                    ErrorCode = sport_client->BodyHeight(Height);
                }
                break;
            case 22100000:
                Last_Operation = "Motion 1 Hello. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->Hello();
                break;
            case 22110000:
                Last_Operation = "Motion 2 Sit. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->Sit();
                break;
            case 22120000:
                Last_Operation = "Motion 3 RiseSit. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->RiseSit();
                break;
            case 22130000:
                Last_Operation = "Motion 4 Dance1. ";
                Last_Operation_Time = this->get_clock()->now();
                ErrorCode = sport_client->Dance1();
                break;
            case 22170000:
                ChattingEnable = 1 - ChattingEnable;
                if(ChattingEnable)
                {
                    Last_Operation = "Listening. ";
                    ErrorCode = Vui_client->SetBrightness(3);
                    publishVoiceChatCommand("start");
                }
                else
                {
                    Last_Operation = "Replying. ";
                    ErrorCode = Vui_client->SetBrightness(0);
                    publishVoiceChatCommand("stop");
                }
                Last_Operation_Time = this->get_clock()->now();
                break;
            default:
                Last_Operation = "Unknown Command. ";
                break;
        }
        
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
