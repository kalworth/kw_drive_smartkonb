五、CAN使用教程
因为用过达妙电机的MIT模式，觉得非常舒服，本人也在dgm的基础上加上了这一部分，但是略有不同。

电机使能指令,发送can地址为电机的can_id
0xFF    0XFF    0XFF    0XFF    0XFF    0XFF    0XFF    0XFC
data[0]    data[1]    data[2]    data[3]    data[4]    data[5]    data[6]    data[7]
电机失能指令,发送can地址为电机的can_id
0xFF    0XFF    0XFF    0XFF    0XFF    0XFF    0XFF    0XFD
data[0]    data[1]    data[2]    data[3]    data[4]    data[5]    data[6]    data[7]
MIT模式，发送can地址为电机的can_id | 0x70,即如果can_id=0x01，那么发送地址为0x71
data[0]    data[1]    data[2]    data[3]    data[4]    data[5]    data[6]    data[7]
pos_ref[7:0]    pos_ref[15:7]    vel_ref[11:4]    vel_ref[0:3] Kp[11:8]    Kp[7:0]    Kd[11:4]    Kd[3:0] iq_ff[11:8]    iq_ff[7:0]
pos_ref：位置给定
vel_ref：速度给定
Kp：位置比例系数
Kd：位置微分系数
iq_ff：q轴电流给定值

以上这些信息按照设置的xxx_max值，从(-max,max)的范围映射到整数，参考达妙电机使用

接收报文，接收报文是发送给电机一次控制报文才返回一次接收报文，接收CAN的地址为can_id | 0x10,即如果电机can_id = 0x01,那么接收的id为0x11;
data[0]    data[1]    data[2]    data[3]    data[4]    data[5]    data[6]    data[7]
can_id    err_flag    pos[15:7]    pos[7:0]    vel[15:7]    vel[15:7]    iq[15:7]    iq[15:7]
can_id：电机can_id
err_flag：电机是否报错,只有1位有效位，0正常，1异常
pos：电机位置
vel：电机速度
iq：电机q轴电流值