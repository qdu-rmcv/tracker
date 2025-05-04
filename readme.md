# 版本说明  

本版本 是 用的 rv 24 开源 ,下位机保持 UL 状态,不需要做任何改变  

## 修改内容  

+ 增加了buff弹道解算部分,在 rv 24 example文件里面的解算c文件基础上修改的  

+ 串口部分修改了 两个发包与一个收包  

+ 增加了buff_interfaces 里面的 BuffSend.msg 以及 速度.msg  

+ 增加了单方向无空气阻力弹道模型,将fly_time 替换为 ftime 

## 新功能   

- [] interfaces 合并  

- [] 收包增加一个 bool 表示切换打armor还是buff  

- [] 收包增加一个 bool 表示录包器 "start" 还是 "stop"  

- [] 去掉 发包 与 收包 里面无意义的 xyz  
 

