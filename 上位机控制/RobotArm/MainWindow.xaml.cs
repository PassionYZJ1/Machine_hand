using RobotArm.servo;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using System.Xml;
using uHand;

namespace RobotArm
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        //命令类型
        public const int CMD_SINGLE_SERVO_MOVE = 2;
        public const int CMD_MULT_SERVO_MOVE = 3;
        public const int CMD_SERVO_STOP = 4;
        public const int CMD_FULL_ACTION_RUN = 6;
        public const int CMD_FULL_ACTION_STOP = 7;
        public const int CMD_FULL_ACTION_ERASE = 8;
        public const int CMD_ACTION_DOWNLOAD = 25;

        //舵机信息
        public const int SERVO_NUM = 6;
        public const int BMP_WIDTH = 100;
        public const int BMP_HEIGHT = 76;


        public const int MAX_ARGS_LENTH = 30;//最大的命令长度


        public const UInt16 UNDEFINECMD = 0xFFFF;//命令buffer默认参数

        //回读舵机情况反馈
        public const int READCOMERRO = -1;
        public const int READDATASUCCESS = 0;
        public const int READDATANONE = 1;


        public const int MAX_ACTION_ITEMS = 255;//动作项最大个数

        public const int PWM_MODE = 0;
        public const int BUS_MODE = 1;


        private ObservableCollection<string> mSerialCom = new ObservableCollection<string>();//串口列表 数据源
        private ObservableCollection<string> actionNumList = new ObservableCollection<string>();//动作组号 数据源
        private ObservableCollection<DownloadInfo> downloadNumList = new ObservableCollection<DownloadInfo>();//动作组号 数据源
        private List<string> baudRate = new List<string>();//波特率
        public Boolean connectStatus = false;//连接状态指示
        public SerialPort ComPort = new SerialPort();//声明一个串口   

        private static bool WaitClose = false;//根据串口状态控制收发

        public UInt16 curActionId = 0;//当前选定的动作组编号

        private readonly TaskScheduler _syncContextTaskScheduler = TaskScheduler.FromCurrentSynchronizationContext();


        private ObservableCollection<ServoAction> m_ServoActions = new ObservableCollection<ServoAction>();//动作列表数据源

        private Boolean stopRunThread = false;//控制在线运行线程标志

        private Boolean needSendAngelChangeFlag = true;//控制是否要根据角度数值变化发送命令 因点击动作列表或者在线运行会显示当前角度值，调整滑竿
                                                       //这个时候会触发angleChangeHandler事件，但是不需要发送命令，用这个变量来控制

        private bool noResponse = false;//控制模式切换是否响应

        private List<ServoView> servosViewList = new List<ServoView>();
        private List<string> fileList = new List<string>();

        public MainWindow()
        {
            InitializeComponent();
            this.servosView.AddHandler(ServoView.AngleChangeEvent, new RoutedEventHandler(this.angleChangeHandler));//添加角度变化的侦听事件

            actionTime.PreviewMouseWheel += actionTimeChange;

            //配置串口参数
            SerialCom.ItemsSource = mSerialCom;
            baudRate.Add("9600");
            baudRate.Add("115200");
            BaudRate.ItemsSource = baudRate;
            BaudRate.SelectedIndex = 0;
            ComPort.ReadTimeout = 200;//串口读超时200ms
            ComPort.WriteTimeout = 200;//串口写超时200ms
            ComPort.ReadBufferSize = 1024;//数据读缓存
            ComPort.WriteBufferSize = 1024;//数据写缓存  

            downloadList.ItemsSource = downloadNumList;
            actionList.ItemsSource = m_ServoActions;
            actionNum.ItemsSource = actionNumList;
            //添加0~230号动作组
            for (int i = 0; i <= 230; i++)
            {
                actionNumList.Add(Convert.ToString(i));
            }
            actionNum.SelectedIndex = 0;

            servosViewList.Add(servo1);
            servosViewList.Add(servo2);
            servosViewList.Add(servo3);
            servosViewList.Add(servo4);
            servosViewList.Add(servo5);
            servosViewList.Add(servo6);
        }


        private void actionTimeChange(object sender, MouseWheelEventArgs e)//滑动鼠标改变动作时间
        {
            int time = Convert.ToInt32(actionTime.Text);
            if (e.Delta > 0)
            {
                if (time < 30000)
                {
                    time++;
                    actionTime.Text = Convert.ToString(time);
                }
            }
            else
            {
                if (time > 0)
                {
                    time--;
                    actionTime.Text = Convert.ToString(time);
                }
            }
            //throw new NotImplementedException();
        }
        private void angleChangeHandler(Object sender, RoutedEventArgs e)
        {
            if (needSendAngelChangeFlag)//手动拖动滑竿的时候才触发，其他情况引起的变化屏蔽
            {
                int id = Convert.ToInt32((e.OriginalSource as ServoView).ServoId);
                int angle = (e.OriginalSource as ServoView).CurAngle;
                sendAngleCmd(id, angle);
            }
        }

        private void ShowSerialCom(object sender, EventArgs e)//显示串口列表
        {
            if (connectStatus)
            {
                return;
            }

            mSerialCom.Clear();
            string[] ports;//可用串口数组
            ports = new string[SerialPort.GetPortNames().Length];//重新定义可用串口数组长度
            ports = SerialPort.GetPortNames();//获取可用串口
            for (int i = 0; i < ports.Length; i++)
                mSerialCom.Add(ports[i]);
        }

        private void OpenComm(object sender, RoutedEventArgs e)
        {
            if (connectStatus)//断开连接
            {
                try//尝试关闭串口
                {
                    WaitClose = true;
                    OpenCloseBtn.Content = "打开串口";//
                    ConnectLed.Source = new BitmapImage(new Uri("/Resources/red.png", UriKind.Relative));//开关状态图片切换为ON
                    connectStatus = false;//串口打开状态字改为true    
                    ComPort.DiscardOutBuffer();//清发送缓存
                    ComPort.DiscardInBuffer();//清接收缓存
                    ComPort.Close();//关闭串口
                }

                catch//如果在未关闭串口前，串口就已丢失，这时关闭串口会出现异常
                {
                    return;//无法关闭串口，提示后直接返回
                }
            }
            else//连接串口
            {
                if (SerialCom.SelectedIndex == -1)
                {
                    MessageBox.Show("您还未选择串口端口");
                    return;
                }
                try//尝试打开串口
                {
                    ComPort.PortName = SerialCom.SelectedValue.ToString();//设置要打开的串口
                    ComPort.BaudRate = Convert.ToInt32(BaudRate.SelectedValue);//设置当前波特率           
                    ComPort.Open();//打开串口

                }
                catch//如果串口被其他占用，则无法打开
                {
                    MessageBox.Show("无法打开串口,请检测此串口是否有效或被其他占用！");
                    return;//无法打开串口，提示后直接返回
                }
                WaitClose = false;
                OpenCloseBtn.Content = "关闭串口";//按钮显示改为“关闭按钮”
                ConnectLed.Source = new BitmapImage(new Uri("/Resources/green.png", UriKind.Relative));//开关状态图片切换为ON
                connectStatus = true;//串口打开状态字改为true             
            }
        }

        private void baudChange(object sender, SelectionChangedEventArgs e)
        {
            ComPort.BaudRate = Convert.ToInt32(BaudRate.SelectedValue);//设置当前波特率    
        }

        private void sendAngleCmd(int id, int value)//发送拖到滑竿引起的角度变化设置命令
        {
            UInt16[] dataSend = new UInt16[MAX_ARGS_LENTH];
            for (int i = 0; i < MAX_ARGS_LENTH; i++)
            {
                dataSend[i] = UNDEFINECMD;
            }
            dataSend[0] = 1;
            dataSend[1] = 0;
            dataSend[2] = 0;
            dataSend[3] = (byte)id;
            dataSend[4] = (byte)(value & 0x00ff);
            dataSend[5] = (byte)(value >> 8);
            makeAndSendCmd(CMD_MULT_SERVO_MOVE, dataSend);
        }

        private void sendActionCmd(UInt16[] args)//发送一帧动作命令
        {
            UInt16[] dataSend = new UInt16[MAX_ARGS_LENTH];
            for (int i = 0; i < MAX_ARGS_LENTH; i++)
            {
                dataSend[i] = UNDEFINECMD;
            }
            dataSend[0] = (UInt16)SERVO_NUM;
            dataSend[1] = (UInt16)(args[0] & 0x00ff);//填入时间
            dataSend[2] = (UInt16)(args[0] >> 8);
            for (int i = 0; i < SERVO_NUM; i++)
            {
                dataSend[3 + i * 3] = (UInt16)(i + 1);
                dataSend[4 + i * 3] = (UInt16)(args[i + 1] & 0x00ff);
                dataSend[5 + i * 3] = (UInt16)(args[i + 1] >> 8);
            }
            makeAndSendCmd(CMD_MULT_SERVO_MOVE, dataSend);
        }

        private void Window_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.LeftButton == MouseButtonState.Pressed)
            {
                DragMove();
            }
        }

        private void makeAndSendCmd(int cmdType, UInt16[] args)//处理参数转换成标准命令协议格式然后发送
        {
            //sendingData = true;
            byte[] dataSend = new byte[50];
            byte lenth;
            dataSend[0] = 0x55;
            dataSend[1] = 0x55;
            dataSend[3] = (byte)cmdType;

            int i = 0;

            while (i <= MAX_ARGS_LENTH && args[i] != UNDEFINECMD)
            {
                dataSend[4 + i] = (byte)args[i];
                i++;
            }
            lenth = (byte)(i + 2);
            dataSend[2] = lenth;//填入长度信息
            WriteData(dataSend, lenth + 2);
        }
        private int WriteData(byte[] buffer, int count)//发送串口数据
        {
            if (!connectStatus)
                return -1;
            try
            {
                ComPort.Write(buffer, 0, count);
            }
            catch
            {
                return -1;
            }
            return 0;
        }

        private int ReadData(byte[] buffer)//读取串口数据，存入buffer中
        {
            byte[] buf = new byte[1024];
            if (!connectStatus || WaitClose)
                return READCOMERRO;
            try
            {

                Array.Clear(buffer, 0, buffer.Length);
                byte[] recBuffer = new byte[ComPort.BytesToRead];//接收数据缓存
                if (recBuffer.Length == 0)
                    return READDATANONE;
                else if (recBuffer.Length > 1024)
                {
                    return READCOMERRO;
                }

                ComPort.Read(buf, 0, recBuffer.Length);
                int i;
                for (i = 0; i < recBuffer.Length - 1; i++)
                {
                    if (buf[i] == 0x55 && buf[i + 1] == 0x55)
                    {
                        break;
                    }
                }
                if (i > recBuffer.Length - 4)
                {
                    return READDATANONE;
                }
                for (int j = 0; j < (buffer.Length < buf.Length - i - 1 ? buffer.Length : buf.Length - i - 1); j++)
                {
                    buffer[j] = buf[i + j];
                }

            }
            catch
            {
                return READCOMERRO;
            }
            return READDATASUCCESS;
        }

        private void setActionListSelectRow(int index)//滚动和设置当前选中行
        {
            DataGridRow row = (DataGridRow)actionList.ItemContainerGenerator.ContainerFromIndex(index);
            if (row == null)
            {
                actionList.UpdateLayout();
                actionList.ScrollIntoView(actionList.Items[index]);
                row = (DataGridRow)actionList.ItemContainerGenerator.ContainerFromIndex(index);
                if (row != null)
                {
                    row.IsSelected = true;
                }
            }
            else
            {
                actionList.UpdateLayout();
                actionList.ScrollIntoView(actionList.Items[index]);
                row.IsSelected = true;
            }
            for (int i = 0; i < m_ServoActions.Count; i++)
            {
                m_ServoActions[i].IndexPath = null;
            }
            m_ServoActions[index].IndexPath = "/Resources/index.png";
        }
        private void addAction(object sender, RoutedEventArgs e)//添加动作操作
        {
            doAddAction(m_ServoActions.Count);
        }

        private void deleteAction(object sender, RoutedEventArgs e)//删除动作
        {
            int selectIndex = actionList.SelectedIndex;
            if (selectIndex != -1)
            {
                m_ServoActions.RemoveAt(selectIndex);
                if (m_ServoActions.Count > 0)
                {
                    if (selectIndex == m_ServoActions.Count)
                    {
                        setActionListSelectRow(m_ServoActions.Count - 1);
                    }
                    else
                    {
                        setActionListSelectRow(selectIndex);
                    }
                    resetItemId();
                }
            }
        }

        private void updateAction(object sender, RoutedEventArgs e)//更新动作
        {
            int selectIndex = actionList.SelectedIndex;
            if (selectIndex != -1)
            {
                List<UInt16> servoAngle = new List<UInt16>();
                for(int i = 0;i < SERVO_NUM; i++)
                {
                     servoAngle.Add(servosViewList[i].CurAngle);
                }
                m_ServoActions[selectIndex].servoAngles = servoAngle;
                m_ServoActions[selectIndex].servoTime = Convert.ToUInt16(actionTime.Text);
            }
        }

        private void insertAction(object sender, RoutedEventArgs e)//插入动作
        {
            int selectIndex = actionList.SelectedIndex;
            if (selectIndex != -1)
            {
                doAddAction(selectIndex);
            }
            else
            {
                doAddAction(m_ServoActions.Count);
            }
            resetItemId();
        }

        private void resetItemId()//重置itemID,这样编号变化后能继续得到正确的值
        {
            for (int i = 0; i < m_ServoActions.Count; i++)
            {
                m_ServoActions[i].itemID = i + 1;
            }
        }

        private void doAddAction(int item)//将当前动作信息加入到列表中
        {
            if (m_ServoActions.Count > MAX_ACTION_ITEMS)
            {
                MessageBox.Show("动作项数超出255的限制，添加失败");
                return;
            }

            ServoAction actionItem = new ServoAction();
            actionItem.servoTime = Convert.ToUInt16(actionTime.Text);
            actionItem.itemID = item + 1;
            List<UInt16> servoAngle = new List<UInt16>();
            for (int i = 0; i < SERVO_NUM; i++)
            {
                 servoAngle.Add(servosViewList[i].CurAngle);
            }
            actionItem.servoAngles = servoAngle;
            m_ServoActions.Insert(item, actionItem);
            //MessageBox.Show("可以弹出请截图联系客服");
            setActionListSelectRow(item);
        }

        private void resetServo(object sender, RoutedEventArgs e)//舵机重置
        {
            needSendAngelChangeFlag = false;
            UInt16[] data = new UInt16[7];
            data[0] = 1000;//固定的复位舵机速度

            for (int i = 0; i < SERVO_NUM; i++)
            {
                servosViewList[i].CurAngle = 1500;
            }
            for (int i = 0; i < SERVO_NUM; i++)
            { 
                data[i + 1] = 1500;
            }
            sendActionCmd(data);
            needSendAngelChangeFlag = true;
        }

        private void openFile(object sender, RoutedEventArgs e)//打开文件
        {
            Microsoft.Win32.OpenFileDialog file_Dialog = new Microsoft.Win32.OpenFileDialog();
            file_Dialog.DefaultExt = ".xml";
            file_Dialog.Filter = "xml file|*.xml";
            if (file_Dialog.ShowDialog() == true)
            {
                setActionList(file_Dialog.FileName);
            }
        }

        private void setActionList(string filename)
        {
            filePath.Text = filename;
            XmlDocument doc = new XmlDocument();
            try
            {
                doc.Load(filename);
                XmlNodeList IDInfo = doc.GetElementsByTagName("ID");
                XmlNodeList moveInfo = doc.GetElementsByTagName("Move");
                XmlNodeList timeInfo = doc.GetElementsByTagName("Time");
                if (IDInfo.Count != moveInfo.Count || IDInfo.Count != timeInfo.Count)
                {
                    MessageBox.Show("XML文件出错");
                    return;
                }
                m_ServoActions.Clear();
                for (int i = 0; i < IDInfo.Count; i++)
                {
                    ServoAction actionItem = new ServoAction();
                    actionItem.itemID = i + 1;
                    string timeTemp = timeInfo[i].InnerText;
                    int posTime = timeTemp.IndexOf('T');
                    string timeStr = timeTemp.Substring(posTime + 1, timeTemp.Length - posTime - 1);
                    actionItem.servoTime = Convert.ToUInt16(timeStr);
                    string[] separator = { " ", "#1 P", "#2 P", "#3 P", "#4 P", "#5 P", "#6 P" };
                    string[] angleStr = moveInfo[i].InnerText.Split(separator, StringSplitOptions.RemoveEmptyEntries);
                    if (angleStr.Length != SERVO_NUM)
                    {
                        MessageBox.Show("XML文件出错");
                        return;
                    }
                    List<UInt16> servoAngle = new List<UInt16>();
                    for (int j = 0; j < SERVO_NUM; j++)
                    {
                        try
                        {
                            servoAngle.Add(Convert.ToUInt16(angleStr[j]));
                        }
                        catch
                        {
                            MessageBox.Show("XML文件出错");
                            return;
                        }
                    }
                    actionItem.servoAngles = servoAngle;
                    m_ServoActions.Add(actionItem);
                }
            }
            catch (Exception k)
            {

                MessageBox.Show(k.Message);
                return;
            }
            setActionListSelectRow(0);
        }

        private void saveFile(object sender, RoutedEventArgs e)//保存文件
        {
            Microsoft.Win32.SaveFileDialog file_Dialog = new Microsoft.Win32.SaveFileDialog();
            file_Dialog.DefaultExt = ".xml";
            file_Dialog.Filter = "xml file|*.xml";
            if (file_Dialog.ShowDialog() == true)
            {
                XmlTextWriter writer = new XmlTextWriter(file_Dialog.FileName, System.Text.Encoding.UTF8);
                writer.Formatting = Formatting.Indented;
                writer.WriteStartDocument();
                writer.WriteStartElement("NewDataSet");
                writer.WriteStartElement("Table");
                for (int i = 0; i < m_ServoActions.Count; i++)
                {
                    writer.WriteElementString("ID", Convert.ToString(i + 1));
                    writer.WriteElementString("Move", getMoveStr(i));
                    writer.WriteElementString("Time", getTimeStr(i));
                }

                writer.WriteEndElement();
                writer.WriteEndElement();
                writer.WriteEndDocument();

                //Write the XML to file and close the writer.
                writer.Flush();
                writer.Close();
            }
        }

        private void runOnlineBT(object sender, RoutedEventArgs e)//在线运行
        {
            if (!connectStatus)
            {
                MessageBox.Show("请先连接控制板");
                return;
            }
            if (m_ServoActions.Count == 0)
            {
                MessageBox.Show("请先添加动作!");
                return;
            }
            if (runOnline.Content.ToString() == "在线运行")
            {
                runOnline.Content = "停止";
                (new Thread(new ThreadStart(runOnlineThread))).Start();
            }
            else if (runOnline.Content.ToString() == "停止")
            {
                stopRunThread = true;
                runOnline.Content = "在线运行";
            }

        }


        public void runOnlineThread()//在线运行处理线程
        {
            for (int i = 0; i < m_ServoActions.Count && !stopRunThread && connectStatus; i++)
            {
                Task.Factory.StartNew(() => selectActionItem(i, true),
                new CancellationTokenSource().Token, TaskCreationOptions.None, _syncContextTaskScheduler).Wait();
                Thread.Sleep(m_ServoActions[i].servoTime);
            }
            Task.Factory.StartNew(() => runOver(),
            new CancellationTokenSource().Token, TaskCreationOptions.None, _syncContextTaskScheduler).Wait();
        }

        private void runOver()//运行结束设置
        {
            if (!connectStatus)
            {
                runOnline.Content = "在线运行";
                MessageBox.Show("连接断开，停止运行");
            }
            else
            {
                if ((Boolean)loopCheck.IsChecked && !stopRunThread)
                {
                    (new Thread(new ThreadStart(runOnlineThread))).Start();
                }
                else
                {
                    runOnline.Content = "在线运行";
                    if (!stopRunThread)
                        MessageBox.Show("运行结束");
                }
            }

            stopRunThread = false;
        }

        private void downLoadBT(object sender, RoutedEventArgs e)//下载动作
        {
            if (!connectStatus)
            {
                MessageBox.Show("请先连接控制板");
                return;
            }
            if (m_ServoActions.Count > 0)
            {
                if(m_ServoActions.Count > 255)
                {
                    MessageBox.Show("动作组项数大于255，只能在线运行，不可以下载到控制板中！");
                }
                else
                {
                    downLoad.IsEnabled = false;
                    (new Thread(new ThreadStart(downloadThread))).Start();//开启下载线程

                }
            }
            else
            {
                MessageBox.Show("请先添加动作!");
            }
        }

        public void downloadThread()//下载线程
        {
            int retryTimes = 20;
            Byte[] RxData = new Byte[20];
            for (int i = 0; i < m_ServoActions.Count && connectStatus; i++)
            {
                UInt16[] dataSend = new UInt16[MAX_ARGS_LENTH];
                for (int k = 0; k < MAX_ARGS_LENTH; k++)
                {
                    dataSend[k] = UNDEFINECMD;
                }
                dataSend[0] = curActionId;
                dataSend[1] = (UInt16)m_ServoActions.Count;
                dataSend[2] = (UInt16)i;
                dataSend[3] = (UInt16)SERVO_NUM;
                dataSend[4] = (UInt16)(m_ServoActions[i].servoTime & 0x00ff);
                dataSend[5] = (UInt16)(m_ServoActions[i].servoTime >> 8);
                for (int j = 0; j < SERVO_NUM; j++)
                {
                    dataSend[6 + 3 * j] = (UInt16)(j + 1);
                    dataSend[7 + 3 * j] = (UInt16)(m_ServoActions[i].servoAngles[j] & 0x00ff);
                    dataSend[8 + 3 * j] = (UInt16)(m_ServoActions[i].servoAngles[j] >> 8);
                }
                makeAndSendCmd(CMD_ACTION_DOWNLOAD, dataSend);
                Thread.Sleep(20);
                int kk;
                for (kk = 0; kk < retryTimes; kk++)
                {
                    if (READDATASUCCESS == ReadData(RxData))
                    {
                        if (RxData[0] == 0x55 && RxData[1] == 0x55 && RxData[2] == 0x02 && RxData[3] == 0x19)//等待的返回数据
                            break;
                    }
                    else
                    {
                        Thread.Sleep(50);
                    }
                }
                if (kk == retryTimes)//等待接收命令超时，下载失败
                {
                    Task.Factory.StartNew(() => downLoadStatus(false),
                        new CancellationTokenSource().Token, TaskCreationOptions.None, _syncContextTaskScheduler).Wait();
                    return;
                }
            }
            Task.Factory.StartNew(() => downLoadStatus(connectStatus),
                 new CancellationTokenSource().Token, TaskCreationOptions.None, _syncContextTaskScheduler).Wait();
        }

        public void downLoadStatus(Boolean flag)//下载结束情况反馈
        {
            if (flag)
            {
                MessageBox.Show("下载成功!");
            }
            else
            {
                MessageBox.Show("下载失败!");
            }
            downLoad.IsEnabled = true;
        }
        private void eraseAllBT(object sender, RoutedEventArgs e)//擦除全部数据
        {
            if (!connectStatus)
            {
                MessageBox.Show("请先连接控制板");
                return;
            }
            (new Thread(new ThreadStart(eraseAllThread))).Start();
        }

        public void eraseAllThread()//擦除线程
        {
            int retryTimes = 20;
            Byte[] RxData = new Byte[20];
            UInt16[] dataSend = new UInt16[MAX_ARGS_LENTH];
            for (int k = 0; k < MAX_ARGS_LENTH; k++)
            {
                dataSend[k] = UNDEFINECMD;
            }
            dataSend[0] = curActionId;
            makeAndSendCmd(CMD_FULL_ACTION_ERASE, dataSend);
            Thread.Sleep(20);
            int j;
            for (j = 0; j < retryTimes; j++)
            {
                if (READDATASUCCESS == ReadData(RxData))
                {
                    if (RxData[0] == 0x55 && RxData[1] == 0x55 && RxData[2] == 0x02 && RxData[3] == 0x8)
                        break;
                }
                else
                {
                    Thread.Sleep(30);
                }
            }
            if (j == retryTimes)//等待接收命令超时
            {
                MessageBox.Show("擦除失败!");
            }
            else
            {
                MessageBox.Show("擦除成功!");
            }
        }

        private void runActionBT(object sender, RoutedEventArgs e)//运行动作组命令
        {
            if (!connectStatus)
            {
                MessageBox.Show("请先连接控制板");
                return;
            }
            UInt16 index = Convert.ToUInt16(actionNum.SelectedValue);
            UInt16[] dataSend = new UInt16[MAX_ARGS_LENTH];
            for (int i = 0; i < MAX_ARGS_LENTH; i++)
            {
                dataSend[i] = UNDEFINECMD;
            }
            dataSend[0] = index;
            dataSend[1] = 0x01;
            dataSend[2] = 0x00;
            makeAndSendCmd(CMD_FULL_ACTION_RUN, dataSend);
        }

        private void stopActionBT(object sender, RoutedEventArgs e)//停止运行动作组命令
        {
            if (!connectStatus)
            {
                MessageBox.Show("请先连接控制板");
                return;
            }
            UInt16[] dataSend = new UInt16[MAX_ARGS_LENTH];
            for (int i = 0; i < MAX_ARGS_LENTH; i++)
            {
                dataSend[i] = UNDEFINECMD;
            }
            makeAndSendCmd(CMD_FULL_ACTION_STOP, dataSend);
        }

        private string getMoveStr(int index)//收集角度信息，形成统一格式填入XML文件中
        {
            string moveStr = "";
            for (int i = 0; i < SERVO_NUM; i++)
            {
                moveStr += "#";
                moveStr += Convert.ToString(i + 1);
                moveStr += " ";
                moveStr += "P";
                moveStr += Convert.ToString(m_ServoActions[index].servoAngles[i]);
                if (i != SERVO_NUM - 1)
                    moveStr += " ";
            }
            return moveStr;
        }

        private string getTimeStr(int index)//获取动作项的时间
        {
            string timeStr = "T";
            timeStr += Convert.ToString(m_ServoActions[index].servoTime);
            return timeStr;
        }

        private void actionIDChange(object sender, SelectionChangedEventArgs e)
        {
            curActionId = Convert.ToUInt16(actionNum.SelectedValue);
        }

        private void selectActionItem(int index, bool flag)//flag为true代表需要滚动列表到该动作项，false代表不需要
        {
            needSendAngelChangeFlag = false;
            if (flag)
                setActionListSelectRow(index);
            UInt16[] data = new UInt16[7];
            data[0] = m_ServoActions[index].servoTime;
            for(int i = 0;i < SERVO_NUM;i++)
            {
                 servosViewList[i].CurAngle = m_ServoActions[index].servoAngles[i];
            }

            for (int i = 0; i < SERVO_NUM; i++)
            {
                data[i + 1] = m_ServoActions[index].servoAngles[i];
            }
            sendActionCmd(data);
            needSendAngelChangeFlag = true;
        }

        private void onCellChange(object sender, EventArgs e)//点击动作项三角图标发送动作命令
        {
            DataGrid dg = sender as DataGrid;
            ServoAction sa = dg.CurrentItem as ServoAction;
            if (sa == null)
                return;
            for (int i = 0; i < m_ServoActions.Count; i++)
            {
                m_ServoActions[i].IndexPath = null;
            }
            if (m_ServoActions.Count >= sa.itemID)
                m_ServoActions[sa.itemID - 1].IndexPath = "/Resources/index.png";
            if (dg.CurrentCell.Column != null)
            {
                if (dg.CurrentCell.Column.DisplayIndex == 0)//选中了触发命令行
                {
                    selectActionItem(sa.itemID - 1, false);
                }
            }
        }

        private void openManyFileClick(object sender, RoutedEventArgs e)
        {
            Microsoft.Win32.OpenFileDialog file_Dialog = new Microsoft.Win32.OpenFileDialog();
            file_Dialog.DefaultExt = ".xml";
            file_Dialog.Filter = "xml file|*.xml";
            file_Dialog.Multiselect = true;
            if (file_Dialog.ShowDialog() == true)
            {
                //openFile(file_Dialog.FileName);
                fileList.Clear();
                foreach (string str in file_Dialog.FileNames)
                {
                    fileList.Add(str);
                    string strNum = str.Substring(str.LastIndexOf("\\") + 1);
                    if (strNum.IndexOf("号") == -1)
                    {
                        MessageBox.Show("文件命名格式不正确,请在文件编号数字后加‘号’字，如1号XXX、2号XXX等");
                    }
                    else
                    {
                        strNum = strNum.Substring(0, strNum.IndexOf("号"));
                        DownloadInfo downloadItem = new DownloadInfo();
                        downloadItem.ActionNum = strNum;
                        downloadNumList.Add(downloadItem);
                    }
                }
            }
        }

        private void openFile(string fileName)
        {
            setActionList(fileName);
        }

        private void downManyFileClick(object sender, RoutedEventArgs e)
        {
            if (!connectStatus)
            {
                MessageBox.Show("请先连接控制板");
                return;
            }
            downLoadMany.IsEnabled = false;
            for (int i = 0; i < fileList.Count; i++)
            {
                openFile(fileList[i]);
                if (!downLoadSteps(Convert.ToUInt16(downloadNumList[i].ActionNum)))
                {
                    downLoadMany.IsEnabled = true;
                    return;
                }
            }
            downLoadMany.IsEnabled = true;
            MessageBox.Show("下载成功!");
        }

        private bool downLoadSteps(UInt16 actGroup)
        {
            int retryTimes = 20;
            Byte[] RxData = new Byte[20];
            for (int i = 0; i < m_ServoActions.Count && connectStatus; i++)
            {
                UInt16[] dataSend = new UInt16[MAX_ARGS_LENTH];
                for (int k = 0; k < MAX_ARGS_LENTH; k++)
                {
                    dataSend[k] = UNDEFINECMD;
                }
                dataSend[0] = actGroup;
                dataSend[1] = (UInt16)m_ServoActions.Count;
                dataSend[2] = (UInt16)i;
                dataSend[3] = (UInt16)SERVO_NUM;
                dataSend[4] = (UInt16)(m_ServoActions[i].servoTime & 0x00ff);
                dataSend[5] = (UInt16)(m_ServoActions[i].servoTime >> 8);
                setActionListSelectRow(i);
                for (int j = 0; j < SERVO_NUM; j++)
                {
                    dataSend[6 + 3 * j] = (UInt16)(j + 1);
                    dataSend[7 + 3 * j] = (UInt16)(m_ServoActions[i].servoAngles[j] & 0x00ff);
                    dataSend[8 + 3 * j] = (UInt16)(m_ServoActions[i].servoAngles[j] >> 8);
                }
                makeAndSendCmd(CMD_ACTION_DOWNLOAD, dataSend);
                Thread.Sleep(20);
                int kk;
                for (kk = 0; kk < retryTimes; kk++)
                {
                    if (READDATASUCCESS == ReadData(RxData))
                    {
                        if (RxData[0] == 0x55 && RxData[1] == 0x55 && RxData[2] == 0x02 && RxData[3] == 0x19)//等待的返回数据
                            break;
                    }
                    else
                    {
                        Thread.Sleep(50);
                    }
                }
                if (kk == retryTimes)//等待接收命令超时，下载失败
                {
                    return false;
                }
            }
            return true;
        }
    }
}
