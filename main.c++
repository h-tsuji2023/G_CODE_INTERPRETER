





#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

//#define OFFLINE_DEBUG
//#define HANDLE_DEBUG


#include "main.h"
//======機械操作情報の保持======
//　サイクルスタート
//   シングルブロック
//    フィードホールド
//    オーバーライド設定
#include "./MachineControl/MachineControl.h"

//=======モーションコントロール=====
//   Gコードの解析をして機械座標を出力する
#include "./MotionControl/MotionControl.h"

//========画面表示=========
#include "./UserInterface/UserInterface.h"

//========NCコントローラメイン=====
#include "./ServoControl/ServoControl.h"
ServoControl servo;

//========機械制御(SUB)=====
//機械固有のIO処理用
#include "./MachineControl/MachineControl_Sub.h"
MachineControl_Sub mc_sub;



//========手動操作========
extern char *manual_mode(void);

//=======Gコード実行モード用======
#include "./G_code_prg.h"


MachineControl mc_status;
MotionControl motion;
UserInterface usr_if;
G_code_prg G_prg;


//==============XY移動モード
float XY_PointData[100][2];
int XY_PointNum=0;
char XY_PointMode_Status=0;


//座標更新用
char Check_Mode;



void* thread1(void* pParam); //スレッド１
void* thread2(void* pParam); //スレッド２
int count1=0;
int count2=0;
pthread_mutex_t mutex; //2つのスレッド間で変数の保護を行う



int
main()
{
	void SetOrigin(void);
	void Manual(void);
	void XY_Point(void);
	void Program(void);

	int i=0;

	char mes[30];

	char Axis[3]={'X','Y','Z'};

	for(i=0;i<100;i++){
		XY_PointData[i][NC_X]=0;
		XY_PointData[i][NC_Y]=0;
	}
		
	


	#ifndef OFFLINE_DEBUG 
	printf("NC_start\n");
	servo.Start();
	printf("Servo_init\n");
	servo.init(&mc_status,&motion);
	
	printf("MC_start\n");
	mc_sub.Start();
	printf("MC_init\n");
	mc_sub.init(&mc_status,&motion);

	

	
	//初期SAP_OFF
	servo.SAP(SAP_OFF);

	//初期リミット設定
	servo.SetLimit(-1000.0,1000.0,
		    -1000.0,1000.0,
		    -1000.0,1000.0);
	servo.LimitMode(0);

	//初期カウンター読み込み
	servo.RetMachineCoodinate(NC_X,COUNTER_CHECK_OFF);
	servo.RetMachineCoodinate(NC_Y,COUNTER_CHECK_OFF);
	servo.RetMachineCoodinate(NC_Z,COUNTER_CHECK_OFF);
	servo.coodinate_renew(COUNTER_CHECK_ON);

	//ターゲット座標を機械座標の初期値で初期化する（スタートから動かないように！）
	servo.SetTarget(NC_X,servo.ReadCoodinate(NC_X));
	servo.SetTarget(NC_Y,servo.ReadCoodinate(NC_Y));
	servo.SetTarget(NC_Z,servo.ReadCoodinate(NC_Z));

	#endif 

	//Gコードプログラム実行モード
	G_prg.init(&mc_status,&motion);

	//ユーザーインターフェイス
	usr_if.init(&mc_status,&motion,&mc_sub);
	usr_if.create_window();


	#ifdef OFFLINE_DEBUG
	motion.set_Coodinate_Offset(1,NC_X,0);
	motion.set_Coodinate_Offset(1,NC_Y,0);
	motion.set_Coodinate_Offset(1,NC_Z,0);
	#else 
	motion.set_Coodinate_Offset(1,NC_X,servo.RetTarget_X());
	motion.set_Coodinate_Offset(1,NC_Y,servo.RetTarget_Y());
	motion.set_Coodinate_Offset(1,NC_Z,servo.RetTarget_Z());
	#endif

	//モーションリセット
	motion.set_G(0);
	motion.set_G(54);
	motion.set_G(90);
	motion.MotionReset();
	motion.ErrorReset();

	
	//オーバーライド設定を０
	mc_status.set_FeedRateHi(0);
	mc_status.set_FeedRateMachining(0);
	mc_status.set_FeedRateMachiningPercent(0);

	//シングルブロック
	mc_status.SetSingleBlock(MC_MODE_ON);
	mc_status.SetCycleStart(MC_MODE_OFF);
	mc_status.SetFeedHold(MC_MODE_ON);

 

	//ステータス初期描画
	usr_if.Status_draw();  
	//初期モード描画
	usr_if.ControlMode_draw(mc_status.ret_ControlMode());



	#ifdef HANDLE_DEBUG
	long int han_x;
	long int han_x_max;
	long int han_x_min;

	while(1){
		//入力読み込み
		usr_if.get_ch();
		//終了
		if(usr_if.ret_ch()=='q' || usr_if.ret_ch()=='Q'){break;}
			mc_sub.ReadHandle();
		han_x=mc_sub.HandleCounter();
		if(han_x_max<han_x){han_x_max=han_x;}
		if(han_x_min>han_x){han_x_min=han_x;}

		sprintf(mes,"%5d  min:%5d max:%5d",han_x,han_x_min,han_x_max);
		usr_if.Message_draw(mes);
		//表示の更新
		usr_if.text_refresh();
		
	}
	#endif





 pthread_t tid1, tid2; // スレッド識別変数

  pthread_mutex_init(&mutex, NULL);
  // スレッドの作成
  pthread_create(&tid1, NULL, thread1, NULL);
  pthread_create(&tid2, NULL, thread2, NULL);
  








	while(1){

		//平均サイクルタイムを計測
		motion.CycleTime();
		mc_status.set_cycle_time(motion.RetCycleTimeAve());


		//操作モード切り替qえを確認
		mc_status.ModeChangeCheckReset();

	#ifndef OFFLINE_DEBUG
		//INOUT読み込み
		mc_sub.ReadIN();
	#endif

		//入力読み込み
		usr_if.get_ch();
		
		//終了
		if(usr_if.ret_ch()=='q' || usr_if.ret_ch()=='Q'){break;}


		switch(usr_if.ret_ch()){
		#ifndef OFFLINE_DEBUG
			case KEY_F(9):
				servo.SAP_change();
				motion.init_XYZ(servo.RetTarget_X(),servo.RetTarget_Y(),servo.RetTarget_Z());
				break;
			//ソフトウェアリミット設定
			case 's':
				servo.LimitMode();
				break;
			case KEY_F(1):
				mc_sub.SpindleOFF();
				break;
			case KEY_F(2):
				mc_sub.SpindleCW();
				break;

		#endif



			//Mode change
			case KEY_NPAGE:
				mc_status.ControlMode_up();
				break;
			case KEY_PPAGE:
				mc_status.ControlMode_down();
				if(mc_status.ret_ControlMode()==SetOriginMode){	
					sprintf(mes,"Set Origin ? (y/n)");
					usr_if.Message_draw(mes);
					usr_if.text_refresh();
					while(1){
						usr_if.get_ch();
						if(usr_if.ret_ch()=='y'){
							break;
						} else if(usr_if.ret_ch()=='n' || usr_if.ret_ch()==KEY_NPAGE) {
							mc_status.ControlMode_up();
							break;
						}
					}
					usr_if.Message_clear();
					
				}
				break;


			////座標関係処理	
			//座標表示切り替え
			case '\\':
				mc_status.coodinate_num_up();
				break;
			case '^':
				mc_status.coodinate_num_down();
				break;


			//送り速度設定
			case KEY_F(5):
				mc_status.set_FeedRateHi_up();
				break;
			case KEY_F(6):
				mc_status.set_FeedRateHi_down();
				break;
			case KEY_F(7):
				mc_status.set_FeedRateMachiningPercent_up();
				break;
			case KEY_F(8):
				mc_status.set_FeedRateMachiningPercent_down();
				break;

			case 'm'://mキー　フィードフォールド
				mc_status.SetFeedHold(MC_MODE_ON);
				break;
			case 'c'://cキー　サイクルスタート
				mc_status.SetCycleStart(MC_MODE_ON);
				break;
			case 'b'://bキー   シングルブロック  
				mc_status.TurnSingleBlock();
				break;
		}
	
		//コントロールモード切替時の初期設定
		if(mc_status.ModeChangeCheck()==ModeChange ||
		  (usr_if.ret_ch()=='o' && mc_status.ret_ControlMode() == ProgramMode) ||
		  (usr_if.ret_ch()=='p' && mc_status.ret_ControlMode() == ProgramMode))   {
			motion.init_XYZ(motion.ret_Target_X(),motion.ret_Target_Y(),motion.ret_Target_Z());
			
			motion.MotionReset();
			motion.ErrorReset();
			//モードチェンジの後はオーバーライド設定を０
			mc_status.set_FeedRateHi(0);
			mc_status.set_FeedRateMachining(0);
			mc_status.set_FeedRateMachiningPercent(0);

			motion.set_G(0);
			motion.set_G(54);
			motion.set_G(90);

			//モードチェンジの後はシングルブロック
			mc_status.SetSingleBlock(MC_MODE_ON);
			mc_status.SetCycleStart(MC_MODE_OFF);
			mc_status.SetFeedHold(MC_MODE_ON);


			//画面消去
			usr_if.Program_clear();
			usr_if.XY_PointData_clear();
			usr_if.Program_clear();
			//画面描画
			usr_if.ControlMode_draw(mc_status.ret_ControlMode());

		}


			
		//コントロールモードによる条件分岐
		switch(mc_status.ret_ControlMode()){
			case SetOriginMode:
				SetOrigin();
				break;
			case ManualMode:
				Manual();
				break;
			case XY_PointMode:
				XY_Point();		
				break;
			case ProgramMode:
				Program();
				break;
		}




	#ifndef OFFLINE_DEBUG
		//座標系誤差表示更新
		usr_if.Coodinate_error_draw(	 servo.ReadTargetError(NC_X)
							,servo.ReadTargetError(NC_Y)
							,servo.ReadTargetError(NC_Z));

	#else

		//座標系誤差表示更新
		usr_if.Coodinate_error_draw(	 motion.ret_TargetError(NC_X)
							,motion.ret_TargetError(NC_Y)
							,motion.ret_TargetError(NC_Z));
	#endif







		//座標系表示更新
		usr_if.Coodinate_draw(mc_status.ret_coodinate_num()
					,motion.ret_Target_X()+motion.ret_Coodinate_Offset(mc_status.ret_coodinate_num(),NC_X)
					,motion.ret_Target_Y()+motion.ret_Coodinate_Offset(mc_status.ret_coodinate_num(),NC_Y)
					,motion.ret_Target_Z()+motion.ret_Coodinate_Offset(mc_status.ret_coodinate_num(),NC_Z));
	



		//ステータス表示
		usr_if.Status_draw();



		

	#ifndef OFFLINE_DEBUG

		//エラーチェック
		switch(servo.ErrorOut()){
			case ERROR__ALL_CLEAR:
				usr_if.Message_clear();
				break;
			case ERROR__COUNTER_ERROR:

				sprintf(mes,"Counter Error");
				usr_if.Message_draw(mes);
				break;
			case ERROR__MOVE_ERROR:
				sprintf(mes,"Move Error");
				usr_if.Message_draw(mes);
				break;
			case ERROR__LIMIT_OVER:
				sprintf(mes,"Soft Limit Over");
				usr_if.Message_draw(mes);
				break;
		}
	#endif


		if(motion.RetError()==MOTION_ERROR){	
			sprintf(mes,"Motion Error");
			usr_if.Message_draw(mes);
		}			


	}

  // スレッド終了待ち
  pthread_join(tid1,NULL);
  pthread_join(tid2,NULL);
  
  pthread_mutex_destroy(&mutex); 

	usr_if.del_window();

	#ifndef OFFLINE_DEBUG
	servo.End();
	mc_sub.End();
	#endif
	 return 0;
}


void SetOrigin(void){
	manual_mode();
	motion.init_XYZ(servo.RetTarget_X(),servo.RetTarget_Y(),servo.RetTarget_Z());
	motion.set_Target_X(servo.RetTarget_X());
	motion.set_Target_Y(servo.RetTarget_Y());
	motion.set_Target_Z(servo.RetTarget_Z());
	servo.Move_Target_XYZ();

	//座標カウンターチェックモードをONにリセット
	Check_Mode=COUNTER_CHECK_ON;
	//機械座標原点設定操作
	if(mc_status.check_coodinate_origin_reset()==0){	
	#ifndef OFFLINE_DEBUG
		if(mc_sub.ret_HomingSW(NC_X)==SW_ON){
			servo.SetOrigin(NC_X);
			motion.init_XYZ(servo.RetTarget_X(),servo.RetTarget_Y(),servo.RetTarget_Z());
			Check_Mode=COUNTER_CHECK_OFF;
		}
		if(mc_sub.ret_HomingSW(NC_Y)==SW_ON){
			servo.SetOrigin(NC_Y);
			motion.init_XYZ(servo.RetTarget_X(),servo.RetTarget_Y(),servo.RetTarget_Z());
			Check_Mode=COUNTER_CHECK_OFF;
		}
		if(mc_sub.ret_HomingSW(NC_Z)==SW_ON){
			servo.SetOrigin(NC_Z);
			motion.init_XYZ(servo.RetTarget_X(),servo.RetTarget_Y(),servo.RetTarget_Z());
			Check_Mode=COUNTER_CHECK_OFF;
		}
		//機械座標更新
		servo.coodinate_renew(Check_Mode);//原点設定したあとはカウンターエラーをチェックしない

	#endif
	}



}


void Manual(void){
	char AxisChoise;
	

	manual_mode();

	//XYZキー入力判定
	AxisChoise=NC_NULL;
	if(usr_if.ret_ch()==('j')){AxisChoise=NC_X;}
	if(usr_if.ret_ch()==('k')){AxisChoise=NC_Y;}
	if(usr_if.ret_ch()==('l')){AxisChoise=NC_Z;}


	//XYZ座標リセット操作
	if(mc_status.check_coodinate_reset()==0 && mc_status.ret_ControlMode()==ManualMode){	
		//モーションコントロールのワーク座標
		motion.set_Coodinate_Offset(mc_status.ret_coodinate_num(),AxisChoise,motion.ret_Target(AxisChoise));

	}



	if(usr_if.ret_ch()==KEY_UP     ){servo.SetTarget_add(NC_Y,-0.001);}
	if(usr_if.ret_ch()==KEY_DOWN) {servo.SetTarget_add(NC_Y, 0.001);}
	if(usr_if.ret_ch()==KEY_RIGHT) {servo.SetTarget_add(NC_X,-0.001);}
	if(usr_if.ret_ch()==KEY_LEFT)   {servo.SetTarget_add(NC_X,0.001);}
	if(usr_if.ret_ch()=='a')            {servo.SetTarget_add(NC_Z,-0.001);}
	if(usr_if.ret_ch()=='z')            {servo.SetTarget_add(NC_Z,0.001);}
	motion.init_XYZ(servo.RetTarget_X(),servo.RetTarget_Y(),servo.RetTarget_Z());
	motion.set_Target_X(servo.RetTarget_X());
	motion.set_Target_Y(servo.RetTarget_Y());
	motion.set_Target_Z(servo.RetTarget_Z());
	servo.Move_Target_XYZ();
}



void XY_Point(void){
	//モードチェンジ後のリセット動作
	if(mc_status.ModeChangeCheck()==ModeChange){
		std::ifstream ifs;
	    	ifs.open("G_CODE_PROGRAM/XY_Point.dat");

		{
			int x,y;
    			int loop=0;
    			while(ifs>>x>>y){

    			//std::cout<<"loop="<<loop;
    			//std::cout<<":x,y="<<x<<","<<y<<std::endl;
			XY_PointData[loop][NC_X]=x;
			XY_PointData[loop][NC_Y]=y;

    			loop++;
			}
			ifs.close();
  		}
	}



	if(usr_if.ret_ch()==KEY_UP || usr_if.ret_ch()==KEY_DOWN || mc_status.ModeChangeCheck()==ModeChange){
		if(usr_if.ret_ch()==KEY_UP    ){if(XY_PointNum<99){XY_PointNum++;}}
		if(usr_if.ret_ch()==KEY_DOWN){if(XY_PointNum>0){XY_PointNum--;}}
		usr_if.XY_PointData_draw(XY_PointNum,XY_PointData[XY_PointNum][NC_X],
								   XY_PointData[XY_PointNum][NC_Y]);
	}
	if(usr_if.ret_ch()=='c'){	
		char G_str[100];
		motion.MotionReset();
		motion.init_XYZ(motion.ret_Target_X(),motion.ret_Target_Y(),motion.ret_Target_Z());
		switch(mc_status.ret_coodinate_num()){
			case 1:
				sprintf(G_str,"G90G00G54X%8.3fY%8.3f;",XY_PointData[XY_PointNum][NC_X],XY_PointData[XY_PointNum][NC_Y]);
				break;
			case 2:
				sprintf(G_str,"G90G00G55X%8.3fY%8.3f;",XY_PointData[XY_PointNum][NC_X],XY_PointData[XY_PointNum][NC_Y]);			
				break;
			case 3:
				sprintf(G_str,"G90G00G56X%8.3fY%8.3f;",XY_PointData[XY_PointNum][NC_X],XY_PointData[XY_PointNum][NC_Y]);			
				break;
			case 4:
				sprintf(G_str,"G90G00G57X%8.3fY%8.3f;",XY_PointData[XY_PointNum][NC_X],XY_PointData[XY_PointNum][NC_Y]);			
				break;
			case 5:
				sprintf(G_str,"G90G00G58X%8.3fY%8.3f;",XY_PointData[XY_PointNum][NC_X],XY_PointData[XY_PointNum][NC_Y]);			
				break;

			case 6:
				sprintf(G_str,"G90G00G59X%8.3fY%8.3f;",XY_PointData[XY_PointNum][NC_X],XY_PointData[XY_PointNum][NC_Y]);			
				break;

		}	

		motion.DataInput(G_str);					
	}
			

	//中間座標移動処理
	//中間座標移動で　TARGET_MOVE →　TARGET_MOVE_END　→　TARGET_NULLに変化していく
	if(mc_status.RetCycleStart()==MC_MODE_ON && servo.ErrorOut()==ERROR__ALL_CLEAR){
		//現在のターゲット座標（機械座標）を送り、次の座標を得る
		motion.CalcTarget(motion.RetCycleTimeAve(),
		mc_status.ret_FeedRateHi(),
		mc_status.ret_FeedRateMachining()*mc_status.ret_FeedRateMachiningPercent()/100.0,
						motion.ret_Target_X(),motion.ret_Target_Y(),motion.ret_Target_Z());

		if(motion.RetError()==MOTION_ERROR_CLEAR){
	#ifndef OFFLINE_DEBUG
			//機械座標入力		
			servo.SetTarget(0,motion.ret_Target_X());
			servo.SetTarget(1,motion.ret_Target_Y());
			servo.SetTarget(2,motion.ret_Target_Z());
	#endif
		}
	}

	//移動完了処理
	//サイクルスタートOFF、早送りゼロにセット
	if((motion.ret_TargetStatus()==TARGET_NULL || motion.ret_TargetStatus()==TARGET_CYCLE_END )
								&& mc_status.RetSingleBlock()==MC_MODE_ON){
		mc_status.SetCycleStart(MC_MODE_OFF);
		mc_status.set_FeedRateHi(0);
	}



	#ifndef OFFLINE_DEBUG
		servo.Move_Target_XYZ();
	#endif
}

void Program(void){
	//======モードチェンジ後のリセット操作===============
	if(mc_status.ModeChangeCheck()==ModeChange || usr_if.ret_ch()=='o'|| usr_if.ret_ch()=='p'){
		//ステップリセット
		G_prg.reset_StepLine();
		//Oキー入力：プログラム呼び出し
		if(usr_if.ret_ch()=='o'){						
			mc_status.up_Program_Num();
		}
		if(usr_if.ret_ch()=='p'){						
			mc_status.down_Program_Num();
		}

		G_prg.program_load(mc_status.ret_Program_Num());	

		//初期画面描画
		usr_if.Program_clear();
		for(int l=-2;l<10;l++){
			usr_if.Program_draw((unsigned int)l+2,G_prg.ret_ProgramLine(l+G_prg.ret_StepLine()));		
		}
		usr_if.Program_draw();
	}
			
	//プログラム実行ループ
	if(motion.ret_TargetStatus()==TARGET_NULL && mc_status.RetCycleStart()==MC_MODE_ON){
		//Gコード文字列を入力
		motion.DataInput(G_prg.ProgramLineOutput());
		mc_status.set_FeedRateMachining(motion.ret_F());
		G_prg.up_StepLine();
	}


	//プログラム表示更新
	if(motion.ret_TargetStatus()==TARGET_MOVE_END ){
		if(G_prg.ret_StepLine()!=G_prg.ret_StepEnd()){
			usr_if.Program_clear();
			for(int l=-2;l<10;l++){
				usr_if.Program_draw((unsigned int)l+2,G_prg.ret_ProgramLine(l+G_prg.ret_StepLine()));		
			}
			usr_if.Program_draw();
		}
		if(G_prg.ret_StepLine()==G_prg.ret_StepEnd()){
			mc_status.SetCycleStart(MC_MODE_OFF);
			mc_sub.SpindleOFF();
		}
	}

	
	//中間座標移動処理
	//中間座標移動で　TARGET_MOVE →　TARGET_MOVE_END　→　TARGET_NULLに変化していく
	if(mc_status.RetCycleStart()==MC_MODE_ON && servo.ErrorOut()==ERROR__ALL_CLEAR){
		//現在のターゲット座標（機械座標）を送り、次の座標を得る
		motion.CalcTarget(motion.RetCycleTimeAve(),
					mc_status.ret_FeedRateHi(),
					mc_status.ret_FeedRateMachining()*mc_status.ret_FeedRateMachiningPercent()/100.0,
					motion.ret_Target_X(),motion.ret_Target_Y(),motion.ret_Target_Z());

		if(motion.RetError()==MOTION_ERROR_CLEAR){
	#ifndef OFFLINE_DEBUG
			//機械座標入力		
			servo.SetTarget(0,motion.ret_Target_X());
			servo.SetTarget(1,motion.ret_Target_Y());
			servo.SetTarget(2,motion.ret_Target_Z());
	#endif
		}
	}
	

	if((motion.ret_TargetStatus()==TARGET_NULL || motion.ret_TargetStatus()==TARGET_CYCLE_END )
								&& mc_status.RetSingleBlock()==MC_MODE_ON){
		mc_status.SetCycleStart(MC_MODE_OFF);
	}
	#ifndef OFFLINE_DEBUG
	servo.Move_Target_XYZ();
	#endif
}




//スレッド１
void* thread1(void* pParam)
{
  int i;
  while(1){
	//終了
	if(usr_if.ret_ch()=='q' || usr_if.ret_ch()=='Q'){break;}
		//表示の更新
		usr_if.text_refresh();
	usleep(3500);


  }
}

//スレッド２
void* thread2(void* pParam)
{
  while(1){
	//終了
	if(usr_if.ret_ch()=='q' || usr_if.ret_ch()=='Q'){break;}


  }
}
