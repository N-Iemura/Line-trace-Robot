#! /usr/bin/env python3

import rospy, cv_bridge	#ROSとOpenCV間でデータを受け渡すためのパッケージ
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy.polynomial.polynomial as P

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv.namedWindow('BGR Image', 1)	#'BGR Image'という名前の画像表示のウィンドウを作成
		#cv.namedWindow('MASK', 1)	#'MASK'という名前の画像表示のウィンドウを作成
		cv.namedWindow('MASKED', 1)	#'MASK'という名前の画像表示のウィンドウを作成
		#cv.namedWindow('MASK_yellow', 1)	
		cv.namedWindow('MASKED_yellow', 1)	
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)	#Image型で画像トピックを購読し，コールバック関数を呼ぶ
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

		self.twist = Twist()	#Twistインスタンス生成

	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')

		#青いラインの検出
		hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)	#色空間の変換(BGR→HSV)
		lower_blue = np.array([90, 64, 0])		#青色の閾値（下限）
		upper_blue = np.array([150, 255, 255])	#青色の閾値（上限）
		mask1 = cv.inRange(hsv, lower_blue, upper_blue)	#閾値によるHSV画像の2値化（マスク画像生成）
		mask2 = cv.inRange(hsv, lower_blue, upper_blue)	#閾値によるHSV画像の2値化（マスク画像生成）
		mask3 = cv.inRange(hsv, lower_blue, upper_blue)	#閾値によるHSV画像の2値化（マスク画像生成）

		#黄色い停止線の検出
		
		lower_yellow1 = np.array([0, 64, 0])		#黄色の閾値（下限）
		upper_yellow1 = np.array([30, 255, 255])	#黄色の閾値（上限）
		maskd1 = cv.inRange(hsv, lower_yellow1, upper_yellow1)	#閾値によるHSV画像の2値化（マスク画像生成）
		mask_yellow = maskd1
		masked_yellow = cv.bitwise_and(image, image, mask = mask_yellow)

		h, w = image.shape[:2]
		RESIZE = (w//3, h//3)
		search_top = (h//4)*3
		search_bot_1 = search_top + 60
		search_bot_2 = search_top + 20	#目の前の線にだけに興味がある→20行分くらいに絞る
		search_bot_3 = search_top - 40

		mask1[0:search_bot_1-20, 0:w] = 0
		mask1[search_bot_1:h, 0:w] = 0

		mask2[0:search_top, 0:w] = 0
		mask2[search_bot_2:h, 0:w] = 0

		mask3[0:search_bot_3, 0:w] = 0
		mask3[search_top-20:h, 0:w] = 0

		mask_yellow[0:590, 0:w] = 0
		masked1 = cv.bitwise_and(image, image, mask = mask1)	#mask画像において，1である部分だけが残る（フィルタに通している）
		masked2 = cv.bitwise_and(image, image, mask = mask2)	#mask画像において，1である部分だけが残る（フィルタに通している）
		masked3 = cv.bitwise_and(image, image, mask = mask3)	#mask画像において，1である部分だけが残る（フィルタに通している）
		masked = masked1 + masked2 + masked3
		cx1, cy1, cx2, cy2, cx3, cy3 = 0, 0, 0, 0, 0, 0

		#分岐
		cx_data = []
		contours1, _ = cv.findContours(mask1, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)	#mask????????1????????????
		contours2, _ = cv.findContours(mask2, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
		contours3, _ = cv.findContours(mask3, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)	#maskにおける1の部分の重心


		contours = [[0, 0, 0], [0, 0, 0]]

		if len(contours1) >= 2:
			contours[0][0] = contours1[0]
			contours[1][0] = contours1[1]
		else:
			contours[0][0] = contours1
			contours[1][0] = contours1

		if len(contours2) >= 2:
			contours[0][1] = contours2[0]
			contours[1][1] = contours2[1]
		else:
			contours[0][1] = contours2
			contours[1][1] = contours2

		if len(contours3) >= 2:
			contours[0][2] = contours3[0]
			contours[1][2] = contours3[1]
		else:
			contours[0][2] = contours3
			contours[1][2] = contours3

		for a1, b2, c3 in contours:
			a1 = np.array(a1)[0]
			b2 = np.array(b2)[0]
			c3 = np.array(c3)[0]
			M1 = cv.moments(a1)	#mask1における1の部分の重心
			M2 = cv.moments(b2)	#mask2における1の部分の重心
			M3 = cv.moments(c3)	#mask3における1の部分の重心
			if M1['m00'] > 0:	#重心が存在する
				cx1 = int(M1['m10']/M1['m00'])	#重心のx座標
				cy1 = int(M1['m01']/M1['m00'])	#重心のy座標
			if M2['m00'] > 0:	#重心が存在する
				cx2 = int(M2['m10']/M2['m00'])	#重心のx座標
				cy2 = int(M2['m01']/M2['m00'])	#重心のy座標
			if M3['m00'] > 0:	#重心が存在する
				cx3 = int(M3['m10']/M3['m00'])	#重心のx座標
				cy3 = int(M3['m01']/M3['m00'])	#重心のy座標
	
			#3???

			data = np.array([[cx1, cy1], [cx2, cy2], [cx3, cy3]])
			k = 0
			datanew = []
			#????????????????????????data????(0,0)??????
			while k<3:
				if data[k][0] == 0 and data[k][1] == 0:
					k+=1
				else:
					datanew.append([data[k][0], data[k][1]])
					k+=1

			datanew = np.array(datanew)
			x = datanew[:, 0]
			y = datanew[:, 1]
			a, b = np.polyfit(x, y, 1)
			y_new = cy2
			x_new = int(((y_new-b)/a)//1)
			cx_data.append(x_new)
			cv.circle(image, (x_new,y_new), 20, (0, 0, 255), -1)	#????????????????
			#右パターン
		if cmd == 0:
			err = max(cx_data) - w//2	#右の重心座標(x)と画像の中心(x)との差
			self.twist.linear.x = 0.3
			#self.twist.angular.z = -float(err)/100	#画像が大きいためか，-1/100では絶対値がまだ十分に大きく，ロボットが暴れてしまう
			self.twist.angular.z = -float(err)/100	#誤差にあわせて回転速度を変化させる（-1/1000がP制御でいうところの比例ゲインにあたる）
			self.cmd_vel_pub.publish(self.twist)
			cv.putText(image, 'Right_Mode', (0, 50), cv.FONT_HERSHEY_PLAIN, 4, (255, 255, 255), 5, cv.LINE_AA)
		#左パターン
		if cmd == 1:
			err = min(cx_data) - w//2	#左の重心座標(x)と画像の中心(x)との差
			self.twist.linear.x = 0.3
			#self.twist.angular.z = -float(err)/100	#画像が大きいためか，-1/100では絶対値がまだ十分に大きく，ロボットが暴れてしまう
			self.twist.angular.z = -float(err)/100	#誤差にあわせて回転速度を変化させる（-1/1000がP制御でいうところの比例ゲインにあたる）
			self.cmd_vel_pub.publish(self.twist)
			cv.putText(image, 'Left_Mode', (0, 50), cv.FONT_HERSHEY_PLAIN, 4, (255, 255, 255), 5, cv.LINE_AA)

			
		#停止線処理
		M_yellow = cv.moments(mask_yellow)
		if M_yellow['m00'] > 0:
			self.twist.linear.x = 0.01
			self.twist.angular.z = 0
			self.cmd_vel_pub.publish(self.twist)
			
		
		#大きすぎるため，サイズ調整
		display_mask = cv.resize(mask2, RESIZE)
		display_masked = cv.resize(masked, RESIZE)
		display_image = cv.resize(image, RESIZE)
		display_mask_yellow = cv.resize(mask_yellow, RESIZE)
		display_masked_yellow = cv.resize(masked_yellow, RESIZE)
		
		#表示
		cv.imshow('BGR Image', display_image)	#'BGR Image'ウィンドウにimageを表示
		#cv.imshow('MASK', display_mask)			#'MASK'ウィンドウにimageを表示
		cv.imshow('MASKED', display_masked)		#'MASKED'ウィンドウにimageを表示
		#cv.imshow('MASK_yellow', display_mask_yellow)			
		cv.imshow('MASKED_yellow', display_masked_yellow)	
		cv.waitKey(3)	#3ミリ秒待つ

cmd = 0
def callback(message):
	global cmd
	if message.data == "Left":
		cmd = 1
	if message.data == "Right":
		cmd = 0

#rospy.init_node('listener')
sub = rospy.Subscriber('/cmd_LR', String, callback)


rospy.init_node('follower')	#'follower'という名前でノードを初期化
follower = Follower()	#Followerクラスのインスタンスを作成（init関数が実行される）
rospy.spin()	#ループ