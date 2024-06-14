using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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

namespace RobotArm.servo
{
    /// <summary>
    /// ServoView.xaml 的交互逻辑
    /// </summary>
    public partial class ServoView : UserControl//舵机控件类
    {
        public ServoView()
        {
            InitializeComponent();
            servoAngleTB.PreviewMouseWheel += mouseAngleChange;
        }

        private UInt16 curAngle;//当前舵机角度值
        public UInt16 CurAngle
        {
            get { return curAngle; }
            set { curAngle = value; servoAngleSlider.Value = value; }
        }

        private UInt16 minAngle;//当前舵机角度最小值
        public UInt16 MinAngle
        {
            get { return minAngle; }
            set { minAngle = value; servoAngleSlider.Minimum = value; }
        }

        private UInt16 maxAngle;//当前舵机角度最大值
        public UInt16 MaxAngle
        {
            get { return maxAngle; }
            set { maxAngle = value; servoAngleSlider.Maximum = value; }
        }

        private void mouseAngleChange(object sender, MouseWheelEventArgs e)//鼠标上下滚动调节舵机角度
        {
            if (e.Delta > 0)
            {
                curAngle = Convert.ToUInt16(servoAngleTB.Text);
                if(curAngle < maxAngle)
                {
                    curAngle++;
                }
                servoAngleTB.Text = Convert.ToString(curAngle);
            }
            else
            {
                curAngle = Convert.ToUInt16(servoAngleTB.Text);
                if (curAngle > minAngle)
                {
                    curAngle--;
                }
                servoAngleTB.Text = Convert.ToString(curAngle);
            }
            OnAngleChange(curAngle);
            //throw new NotImplementedException();
        }
        //定义ID属性
        public string ServoId//舵机的ID
        {
            get { return (String)GetValue(ServoIdProperty); }
            set { SetValue(ServoIdProperty, value); }
        }

        // Using a DependencyProperty as the backing store for ServoId.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty ServoIdProperty =
            DependencyProperty.Register("ServoId", typeof(String), typeof(ServoView), new PropertyMetadata("TextBlock", new PropertyChangedCallback(OnTextChanged)));
        static void OnTextChanged(object sender, DependencyPropertyChangedEventArgs args)
        {
            ServoView source = (ServoView)sender;
            source.id.Text = "ID:" + args.NewValue as string;
        }

        //定义路由事件
        public static readonly RoutedEvent AngleChangeEvent =
            EventManager.RegisterRoutedEvent("AngleChange", RoutingStrategy.Bubble, typeof(RoutedEventHandler), typeof(ServoView));

        public event RoutedEventHandler AngleChange
        {
            add
            {
                this.AddHandler(AngleChangeEvent, value);
            }

            remove
            {
                this.RemoveHandler(AngleChangeEvent, value);
            }
        }

        public void OnAngleChange(UInt16 value)//角度变化，将该信息发送出去，MainWindow的servosView侦听了该事件
        {
            this.curAngle = value;
            servoAngleSlider.Value = value;
            RoutedEventArgs arg = new RoutedEventArgs(AngleChangeEvent,this);

            this.RaiseEvent(arg);
        }

        private void angleChange(object sender, TextChangedEventArgs e)
        {
            TextBox textBox = sender as TextBox;
            OnAngleChange(Convert.ToUInt16(textBox.Text));
        }
    }
}
