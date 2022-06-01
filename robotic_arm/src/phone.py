# coding=utf-8
# 该代码示例适用于Python2
import urllib
import urllib2
import hashlib
import rospy
from std_msgs.msg import String

def md5(str):
    import hashlib
    m = hashlib.md5()
    m.update(str)
    return m.hexdigest()

class Phone():
    def __init__(self):
        rospy.init_node("email_node")
        self.sub = rospy.Subscriber("email_phone", String, self.callback, queue_size=5)
    
    def callback(self,data):
        statusStr = {
            '0': '短信发送成功',
            '-1': '参数不全',
            '-2': '服务器空间不支持,请确认支持curl或者fsocket,联系您的空间商解决或者更换空间',
            '30': '密码错误',
            '40': '账号不存在',
            '41': '余额不足',
            '42': '账户已过期',
            '43': 'IP地址限制',
            '50': '内容含有敏感词'
            }
        smsapi = "http://api.smsbao.com/"
        # 短信平台账号
        user = 'tinyzoe'
        # 短信平台密码
        password = md5('2806be6289304ff6a086abc0ee2c88eb')
        # 要发送的短信内容
        content = data.data
        # content = '同济大学：苹果1个'
        # 要发送短信的手机号码
        phone = '18377198012'
        sendurl = self.smsapi + 'sms?'
        data = urllib.urlencode({'u': user, 'p': password, 'm': phone, 'c': content})
        req = urllib2.Request(sendurl, data)
        response = urllib2.urlopen(req)
        the_page = response.read()
        # print statusStr[the_page]


if __name__ == "__main__":
    phone = Phone()
    rospy.spin()