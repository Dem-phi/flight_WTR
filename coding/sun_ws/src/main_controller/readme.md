# 开发指南

## 1 主要组件

### 1.1 计划表 Schedule Table

```c++
    FSM FSMachine(nh);
    FSMachine.build_ScheduleTable(
    	sun::TAKEOFF,
        sun::DETECTING,
        sun::DOCKING, sun::DOCKING_CIRCLE, 3, 120, 100,
        sun::TAKEOFF,
        sun::DETECTING,
        sun::LANDING,
        sun::END
    );
    FSMachine.set_timer();
```

在main函数中，只需要创建状态机，然后建立计划表，其中填入流程，就可以生成流程表运行。这本来是个函数传参的过程，为了足够明显，把它写开了，每一行第一个参数是状态，后续参数是这个状态需要的参数。例如DOCKING模式就需要知道需要对准圆形还是方形。这个函数的参数是动态的，所以可以随意修改所需参数。

他最大的好处是灵活性。在DEBUG阶段，可以只修改计划表来运行其中某一个状态。

其中sun::END状态是没有意义的，它的作用是表示已经输入完毕，函数不会再继续读取参数。

### 1.2 工作者 State Workers

计划表的本质就是依次创建工作者实例去工作。每一个状态对应一个工作者，一个类，头文件写在/include/StateWorkers里面。

```c++
#ifndef _MANUALWORKER_
#define _MANUALWORKER_

#include <StateWorker.h>

class ManualWorker:public StateWorker{
public:
    ros::NodeHandle nh;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    ManualWorker(ros::NodeHandle &nh);
    ~ManualWorker();
};

ManualWorker::ManualWorker(ros::NodeHandle &nh){
    this->nh = nh;
}

ManualWorker::~ManualWorker(){}

void ManualWorker::run(StateInfo state_info){
    cout << "ManualWorker is running" << endl;
    return;
}

bool ManualWorker::is_finished(){
    cout << "ManualWorker is finished" << endl;
    return true;
}

#endif
```

其中run()函数和is_finished()函数必须重写实现。上面是一个例子。

### 1.3 主循环 loop

```c++
void FSM::loop(const ros::TimerEvent &){
    // running by schedule table
    if(this->Workers[this->flow]->is_finished()){
        this->flow ++;
        if(this->flow == this->Workers.size()){
            ROS_INFO("Finish ScheduleTable!");
            exit(0);
        }
    }else{
        this->Workers[this->flow]->run(this->state_info);
    }
    // update state_info
    update_bluetooth_data(&this->state_info);
    update_serial_data(&this->state_info);
}
```

主体循环其实就是

- 检查现在运行到那个流程了
- 判断这个流程是否已经完成is_finished()函数
  - 如果完成，则执行下个流程，如果流程执行完了，就会退出程序，**所以计划表一定要用LAND结尾**
- 执行那个流程的工作者的run()函数
- 更新状态

### 1.4 状态信息 State Info

为了避免信息重复订阅或者过于繁琐，所有的信息交换都定义在了MsgJar.h中，主要包括结构体StateInfo，其中包括了所有飞机会用到的状态信息（因为功能都被分散到了单独的Worker中，所以说很多信息都不需要了，交给每个Worker去完成就可以了）

```c++
typedef struct StateInfo{
    bool connected;                     // is connected
    bool armed;                         // is unlocked
    string mode;                        // flight mode, OFFBOARD eg.
    geometry_msgs::Vector3 linear;      // linear velocity
    geometry_msgs::Vector3 angular;     // angular velocity
    float height;                       // height data from laser
}StateInfo;
```

PX4给出的数据会在主循环中的回调函数里更新，其他的数据，比如串口数据和蓝牙数据要用下面两个函数更新

```c++
void update_serial_data(StateInfo* state_info){
    /* code */
    return;
}

void update_bluetooth_data(StateInfo* state_info){
    /* code */
    return;
}
```

都还没有实现。已经把状态机的state_info指针传给它了，只需要把这个指针里要更新的数据更新掉就ok。

Workers的run函数都会拿到这个state_info，所以哪怕他们都是独立于状态机的单独的类，也可以拿到必要的数据信息。

### 1.5 常规定义和常规参数 common_defination & common_parameters

在/include/StateWorkers里面可以看到这两个头文件，其中common_defination定义了枚举变量，common_parameters定义了非枚举变量，我把很多算法或者流程的参数都定义到这里，这样需要修改的时候就不需要找。

### 1.6 工具箱 Math Tools

我把所有用到的数学算法还有工具都写在这个里面，常用的工具都可以写在里面方便调用。

**首先**，其中可能用的最多的就是PIDcontroller，下面是使用方法，非常方便，经常会用到。

```c++
// 定义PID参数，类型是Eigen::Vector3f，一般我会把这个参数定义到common_parameters里面，方便修改
Eigen::Vector3f PID_params(6, 1, 0.2);

// 创建PIDcontroller类，构造函数需要给它PID参数
PIDcontroller model_PID( PID_params );

// 虽然创建的时候要给PID参数，但是也可以创建完了再改，只要修改它的PID_params变量就可以
model_PID.PID_params(Eigen::Vector3f(5, 1.2, 0.8));

// 使用的时候首先要计算误差向量，是一个二维向量，如果你需要用一维的PID，只需要把一个维度弄成0，就可以了，假设我们已经通过某种方式得到了此时的误差
Eigen::Vector2f Err = getErr();
// 只需要用run()函数，返回的input向量就是计算的输入向量
Eigen::Vector2f input = model_PID.run(Err);
```

创建完成后，每次循环只需要用一行代码就可以算出结果。

**另外**，重写了一个有约束的速度发布函数

```c++
void publish_vel(ros::Publisher* pub, geometry_msgs::TwistStamped msg){
    float linear_speed = sqrt(
        msg.twist.linear.x*msg.twist.linear.x +
        msg.twist.linear.y*msg.twist.linear.y + 
        msg.twist.linear.z*msg.twist.linear.z 
    );
    float angular_speed = sqrt(
        msg.twist.angular.x*msg.twist.angular.x + 
        msg.twist.angular.y*msg.twist.angular.y + 
        msg.twist.angular.z*msg.twist.angular.z 
    );
    if(linear_speed > sun::MAX_LINEAR_VEL){
        float shrink_scale = sun::MAX_LINEAR_VEL/linear_speed;
        msg.twist.linear.x *= shrink_scale;
        msg.twist.linear.y *= shrink_scale;
        msg.twist.linear.z *= shrink_scale;
    }
    if(angular_speed > sun::MAX_ANGULAR_VEL){
        float shrink_scale = sun::MAX_ANGULAR_VEL/angular_speed;
        msg.twist.angular.x *= shrink_scale;
        msg.twist.angular.y *= shrink_scale;
        msg.twist.angular.z *= shrink_scale;
    }
    pub->publish(msg);
    return;
}
```

```c++
float MAX_LINEAR_VEL = 3;
float MAX_ANGULAR_VEL = 1;
```

首先在common_parameters中定义了飞机最大的线速度和角速度，publish_vel函数会先检查要发送的速度向量是否超过了最大速度限制，如果超过了，就会按比例压缩使得发送速度的大小变成约束最大速度，然后再发出去。

所以我们代码里面发送速度信息，都用这个发，就比较安全。

## 2 开发一个工作者

### 2.1 找到要开发的头文件并且写好注释

所有需要用的工作者我都创建完了，都是未开发状态，也就是说所有头文件，除非我已经写了，不然除了名字以外，应该都长这样

```c++
#ifndef _DETECTINGWORKER_
#define _DETECTINGWORKER_

#include <StateWorker.h>
class DetectingWorker:public StateWorker{
public:
    ros::NodeHandle nh;

    virtual void run(StateInfo state_info);
    virtual bool is_finished();

    DetectingWorker(ros::NodeHandle &nh);
    ~DetectingWorker();
};

DetectingWorker::DetectingWorker(ros::NodeHandle &nh){
    this->nh = nh;
}

DetectingWorker::~DetectingWorker(){}

void DetectingWorker::run(StateInfo state_info){
    cout << "DetectingWorker is running" << endl;
    return;
}

bool DetectingWorker::is_finished(){
    cout << "DetectingWorker is finished" << endl;
    return true;
}

#endif
```

然后要在类名字的上面写个注释，写一下他完成的工作，还有构造函数需要输入什么参数

```c++
/* 
@brief Docking worker can locate the center of the given figure accurately and finish the docking work.
@param ros::NodeHandle &nh; 
@param int type; enum param deciding the docking category of figure
@param int level: multi-level selection, you can choose 3, 2, 1, Default 1 when receiving wrong command
@param int bias_x; bias vector. Docking worker will try to attach image_center+bias_vector to figure_center
@param int bias_y;
 */
class DockingWorker:public StateWorker{}
```



### 2.2 修改构造函数和状态机

每个worker有不同的需要，比如detecting需要有不同的大致方位，这些都要传入到构造函数中，因为run函数和is_finished函数的参数都**不能够修改**，所以一切必要的信息都要从构造函数填进去。

```c++
DockingWorker(ros::NodeHandle &nh, int type, int level, int bias_x, int bias_y);
```

然后就要修改状态机类的计划表建立函数

```c++
void FSM::build_ScheduleTable(int Schedule, ...)
```

因为它的参数是动态输入的，所以需要明确对应的状态到底需要传入哪些参数

```c++
case sun::DOCKING:{
                int type = va_arg(arg_ptr, int);
                int level = va_arg(arg_ptr, int);
                int bias_x = va_arg(arg_ptr, int);
                int bias_y = va_arg(arg_ptr, int);
                DockingWorker* tmp_worker = new DockingWorker(this->nh, type, level, bias_x, bias_y);
                this->Workers.push_back((StateWorker*)tmp_worker);
                break;
            }
```

这是一个例子，因为修改了Worker的构造函数，所以创建类的时候需要多输入4个int类型的参数，int type = va_arg(arg_ptr, int);的意思其实就是，函数在读取到sun::DOCKING这个参数以后，再往后读取一个int类型的参数，并且赋值给type，这样就实现动态参数了。然后把这些参数传给类的构造函数，就完成了。



### 2.3 重写run函数和is_finished函数

run函数就是主循环在执行这个流程的时候，循环执行的函数，它必须且只能得到一个参数**state_info**，他就是那个数据结构体。

is_finished函数就是判断这个流程是否已经执行完了，一旦它返回true，那就会跳到下一个流程去执行

```c++
void DockingWorker::run(StateInfo state_info)
```

```c++
bool DockingWorker::is_finished()
```

run管做什么事，is_finished管判断是否完成

纯靠这俩就可以完成一个状态功能的实现。

有任何需要的数据都最好添加到**StateInfo**结构体里面，因为每个类都脱离了状态机，所以说在类里面创建subscriber，我并不确定他能不能触发回调函数。**我故意弄成这样的**，这样可以减少杂乱繁琐的数据交换。

### 2.4 然后就写好了

在写的时候不可避免会遇到一些需要调整的参数，因为我在家里没有办法实地考察，或者说代码编辑阶段还没有开始实地测试，所以很多参数都需要完善和修改。有一个developLOG.md文件，记录了所有还没有弄完的参数，防止遗忘，开发的时候最好往里面填一下。

## 3 还需要完成的

- DetectingWorker：朝着给定的方位缓慢飞行，并且寻找圆或者方块，如果飞到一定的时间还没有找到，就原地暂停不动。
- LandingWorker：降落，并且降落完了还要让飞机锁定，锁定之后exit(0)，就是这么绝，确保降落后万无一失
- ManualWorker：手动模式，可以把遥控器发送的数据信息写到StateInfo结构体里头
- OffloadingWorker：与stm32通讯，等待卸货，卸货完成后切换到下一步。
- TakeoffWorker：飞机解锁，并且飞到指定高度
- WaitingWorker：发送0速度，原地不动
- MsgJar中的蓝牙通讯数据更新函数
- MsgJar中的串口通讯数据更新函数
- 参数调试