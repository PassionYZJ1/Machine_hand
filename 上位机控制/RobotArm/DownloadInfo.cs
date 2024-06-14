using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;

namespace uHand
{
    class DownloadInfo : INotifyPropertyChanged//该类作为动作列表项的基本数据类型
    {
        private string actionNum;

        public event PropertyChangedEventHandler PropertyChanged;

        public string ActionNum
        {
            get { return actionNum; }
            set
            {
                actionNum = value;
                if (this.PropertyChanged != null)
                {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("ActionNum"));
                }
            }
        }
    }
}
