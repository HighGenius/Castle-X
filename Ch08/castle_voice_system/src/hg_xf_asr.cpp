/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

using namespace std;

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096
#define ASRCMD 1
bool asr_flag = false;
bool record_flag = true;
string result = "";

/* Upload User words */
/*
static int upload_userwords()    //上传用户词典
{
	char*			userwords	=	NULL;
	size_t			len			=	0;
	size_t			read_len	=	0;
	FILE*			fp			=	NULL;
	int				ret			=	-1;

	fp = fopen("userwords.txt", "rb");    //在bin目录下的userwords.txt，用户词典里面的编辑格式必须合法
	if (NULL == fp)										
	{
		printf("\nopen [userwords.txt] failed! \n");
		goto upload_exit;
	}

	fseek(fp, 0, SEEK_END);    //指针跳转到文本末端
	len = ftell(fp);     //知道文本大小
	fseek(fp, 0, SEEK_SET);  	//跳转回文本开头				
	
	userwords = (char*)malloc(len + 1);
	if (NULL == userwords)
	{
		printf("\nout of memory! \n");
		goto upload_exit;
	}

	read_len = fread((void*)userwords, 1, len, fp); 
	if (read_len != len)
	{
		printf("\nread [userwords.txt] failed!\n");
		goto upload_exit;
	}
	userwords[len] = '\0';
	
	MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret); //ÉÏ´«ÓÃ»§´Ê±í
	if (MSP_SUCCESS != ret)
	{
		printf("\nMSPUploadData failed ! errorCode: %d \n", ret);
		goto upload_exit;
	}
	
upload_exit:
	if (NULL != fp)
	{
		fclose(fp);
		fp = NULL;
	}	
	if (NULL != userwords)
	{
		free(userwords);
		userwords = NULL;
	}
	
	return ret;
}
*/
//该函数功能是当asr有文本返回时，将文本打印出来，因此适合作为asr动作的标志
static void show_result(char *str, char is_over)
{
	printf("\rResult: [ %s ]", str);
	if(is_over)
		putchar('\n');
	string s(str);
	result = s;
	asr_flag = true;
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);
		show_result(g_result, is_last);
	}
}
void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("Start Listening...\n");
}
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
	{
		printf("\nSpeaking done \n");
		record_flag = false;
	}
	else
		printf("\nRecognizer error %d\n", reason);
}

/* demo send audio data from a file */
/*
static void demo_file(const char* audio_file, const char* session_begin_params)
{
	int	errcode = 0;
	FILE*	f_pcm = NULL;
	char*	p_pcm = NULL;
	unsigned long	pcm_count = 0;
	unsigned long	pcm_size = 0;
	unsigned long	read_size = 0;
	struct speech_rec iat;
	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	if (NULL == audio_file)
		goto iat_exit;

	f_pcm = fopen(audio_file, "rb");
	if (NULL == f_pcm)
	{
		printf("\nopen [%s] failed! \n", audio_file);
		goto iat_exit;
	}

	fseek(f_pcm, 0, SEEK_END);
	pcm_size = ftell(f_pcm);
	fseek(f_pcm, 0, SEEK_SET);

	p_pcm = (char *)malloc(pcm_size);
	if (NULL == p_pcm)
	{
		printf("\nout of memory! \n");
		goto iat_exit;
	}

	read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm);
	if (read_size != pcm_size)
	{
		printf("\nread [%s] error!\n", audio_file);
		goto iat_exit;
	}

	errcode = sr_init(&iat, session_begin_params, SR_USER, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed : %d\n", errcode);
		goto iat_exit;
	}

	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("\nsr_start_listening failed! error code:%d\n", errcode);
		goto iat_exit;
	}

	while (1)
	{
		unsigned int len = 10 * FRAME_LEN; // 200ms audio 
		int ret = 0;

		if (pcm_size < 2 * len)
			len = pcm_size;
		if (len <= 0)
			break;

		ret = sr_write_audio_data(&iat, &p_pcm[pcm_count], len);

		if (0 != ret)
		{
			printf("\nwrite audio data failed! error code:%d\n", ret);
			goto iat_exit;
		}

		pcm_count += (long)len;
		pcm_size -= (long)len;		
	}

	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("\nsr_stop_listening failed! error code:%d \n", errcode);
		goto iat_exit;
	}

iat_exit:
	if (NULL != f_pcm)
	{
		fclose(f_pcm);
		f_pcm = NULL;
	}
	if (NULL != p_pcm)
	{
		free(p_pcm);
		p_pcm = NULL;
	}

	sr_stop_listening(&iat);
	sr_uninit(&iat);
}
*/
/* demo recognize the audio from microphone */
static void demo_mic(const char* session_begin_params)
{
	int errcode;
	int i = 0;

	struct speech_rec iat;

	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	/* demo 15 seconds recording */
	while(record_flag && i<=15)
	{
		sleep(1);
		i++;
	}
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("stop listening failed %d\n", errcode);
	}

	sr_uninit(&iat);
}

//语音识别主功能函数
void asr_Process()
{
	int ret = MSP_SUCCESS;
	//int upload_on =	1; /* whether upload the user word */
	/* login params, please do keep the appid correct */
	const char* login_params = "appid = 5b6d44ed, work_dir = .";
	//int aud_src = 0; /* from mic or file */

	/*
	* See "iFlytek MSC Reference Manual"
	*/
	const char* session_begin_params =
		"sub = iat, domain = iat, language = zh_cn, "
		"accent = mandarin, sample_rate = 16000, "
		"result_type = plain, result_encoding = utf8";

	/* Login first. the 1st arg is username, the 2nd arg is password
	 * just set them as NULL. the 3rd arg is login paramertes 
	 * */
	ret = MSPLogin(NULL, NULL, login_params);    //登录
	if (MSP_SUCCESS != ret)	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		goto exit; // login fail, exit the program
	}
/*
	printf("Want to upload the user words ? \n0: No.\n1: Yes\n");    //设置是否上传用户词典，没购买只能选0
	scanf("%d", &upload_on);
	if (upload_on)
	{
		printf("Uploading the user words ...\n");
		ret = upload_userwords();
		if (MSP_SUCCESS != ret)
			goto exit;	
		printf("Uploaded successfully\n");
	}


	printf("Where the audio comes from?\n"     //现场录音还是拿音频文件？
			"0: From a audio file.\n1: From microphone.\n");
	scanf("%d", &aud_src);
	if(aud_src != 0) {
		printf("Demo recognizing the speech from microphone\n");

		printf("Speak in 15 seconds\n");
*/
		demo_mic(session_begin_params);
/*
		printf("15 sec passed\n");
	} else {
		printf("Demo recgonizing the speech from a recorded audio file\n");
		demo_file("wav/iflytek02.wav", session_begin_params); 
	}
	*/
exit:
	MSPLogout(); // Logout...

}

void asrCallback(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO_STREAM("you are speaking...");
	if(msg->data == ASRCMD)
	{
		asr_Process();
	}
}


/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */
int main(int argc, char** argv)
{

	ros::init(argc, argv, "castle_xf_asr_node");    //初始化节点，向节点管理器注册
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/voice/castle_xf_asr_topic", 1, asrCallback);

	ros::Publisher pub = n.advertise<std_msgs::String>("/voice/castle_tl_nlu_topic", 3);

	ros::Rate loop_rate(10);    //10Hz循环周期
	while(ros::ok())
	{
		if(asr_flag)
		{
			std_msgs::String msg;
			msg.data = result;    //将asr返回文本写入消息，发布到topic上
			pub.publish(msg);
			asr_flag = false;
			record_flag = true;
		}
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}


