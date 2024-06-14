using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotArm
{
    class ServoAction: INotifyPropertyChanged//该类作为动作列表项的基本数据类型
    {
        private string indexPath;
        private UInt16 time;//动作时间
        private int item;//动作项的ID标识
        private List<UInt16> servoAngle = new List<UInt16>() ;//动作项6个舵机的角度值

        public event PropertyChangedEventHandler PropertyChanged;

        public UInt16 servoTime
        {
            get { return time; }
            set
            {
                time = value;
                if(this.PropertyChanged != null)
                {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("servoTime"));
                }
            }
        }

        public string IndexPath
        {
            get { return indexPath; }
            set
            {
                indexPath = value;
                if (this.PropertyChanged != null)
                {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("IndexPath"));
                }
            }
        }


        public List<UInt16> servoAngles
        {
            get { return servoAngle; }
            set
            {
                servoAngle = value;
                if (this.PropertyChanged != null)
                {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("servoAngles"));
                }
            }
        }

        
        public int itemID
        {
            get { return item; }
            set
            {
                item = value;
                if (this.PropertyChanged != null)
                {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("itemID"));
                }
            }
        }
        
    }
}
