// NTRViewer.cpp : 定义控制台应用程序的入口点。
//
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "getopt.h"
#include <turbojpeg.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define PACKET_SIZE (1448)
#define WINAPI

float topScaleFactor = 1;
float botScaleFactor = 1;
int screenWidth;
int screenHeight;
int layoutMode = 1;
int logLevel = 1;

int global_jpeg_quality = 100;

SDL_Window* mainWindow;
SDL_Renderer* renderer;
SDL_Texture* topTexture;
SDL_Texture* botTexture;

tjhandle decompressHandle;

u8* jdecOutputBuf;
u8* jdecInputBuf;
u32 jdecRemainOut, jdecRemainIn, jdecWidth, jdecHeight;
u8 jdecWorkBuffer[8192 * 1024];

u8 recvBuffer[2][2][1444 * 140 * 3];


u8 topBuffer[400 * 240 * 3 * 3];
u8 botBuffer[320 * 240 * 3 * 3];
u8 decompressBuffer[2][400 * 240 * 3 * 3];

int totalCount = 0;
int badCount = 0;
int recoverCount = 0;
float totalCompressRatio = 0;
int compressCount = 0;
int lastTime;

int topRequireUpdate = 0;
int botRequireUpdate = 0;
int lastFormat = -1;

u8 buf[2000];
u8 trackingId[2];
int newestBuf[2], bufCount[2][2], isFinished[2][2];
int bufTarget[2][2];

int uncompressedTargetCount[2] = {  107 , 133};


char activateStreamingHost[256];

int fullScreenMode = 0;
int hideCursor = 0;

int priorityMode = 0;
int priorityFactor = 5;
int jpegQuality = 80;
float qosValue = 30.0;

void activateStreaming(char* host);

void printTime() {
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[128];
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "[%H:%M:%S] ", timeinfo);
	printf("%s", buffer);
}

int logI(const char *format, ...)
{
	va_list ap;
	int retval;
	va_start(ap, format);
	printTime();
	retval = vprintf(format, ap);
	va_end(ap);
	return retval;
}

int logV(const char *format, ...)
{
	va_list ap;
	if (logLevel < 2) {
		return 0;
	}
	int retval;
	va_start(ap, format);
	printTime();
	retval = vprintf(format, ap);
	va_end(ap);
	return retval;
}


void transBuffer(u16* dst, u16* src, int width, int height, int format) {
	int blockSize = 16;
	u16* blockStart;

	for (int x = 0; x < width; x += blockSize) {
		for (int y = 0; y < height; y += blockSize){
			blockStart = dst + x * height + y;
			for (int i = 0; i < blockSize; i++) {
				for (int j = 0; j < blockSize; j++){
					*(blockStart + i * height + j) = *src;
					src++;
				}
			}
		}
	}
}

void convertBuffer(u8* dst, u8* src, int width, int height, int format) {
	u32 x, y;
	u8 *dp, *sp;
	u32 bytesPerRow = width * 3;
	dp = dst;
	sp = src;

	for (x = 0; x < width; x++) {
		dp = dst + bytesPerRow * (height - 1) + 3 * x;
		for (y = 0; y < height; y++) {
			dp[0] = sp[0];
			dp[1] = sp[1];
			dp[2] = sp[2];
			sp += 3;
			dp -= bytesPerRow;
		}
	}
}

int getIdDiff(u8 now, u8 before) {
	if (now >= before) {
		return now - before;
	}
	else {
		return (u32)now + 256 - before;
	}
}

void advanceBuffer(u8 isTop) {
	totalCount += 1;
	if (!isFinished[isTop][!newestBuf[isTop]]) {
		logV("1 bad frame dropped\n");
		badCount += 1;
	}
	newestBuf[isTop] = !newestBuf[isTop];
	bufCount[isTop][newestBuf[isTop]] = 0;
	isFinished[isTop][newestBuf[isTop]] = 0;
	bufTarget[isTop][newestBuf[isTop]] = 9999;//isTop ? 9999 : uncompressedTargetCount[isTop];


}

void resetBuffer(u8 isTop) {
	newestBuf[isTop] = 0;
	bufCount[isTop][0] = 0;
	bufCount[isTop][1] = 0;
	isFinished[isTop][0] = 0;
	isFinished[isTop][1] = 0;
	bufTarget[isTop][0] = 9999; //isTop ? 9999 : uncompressedTargetCount[isTop];
	bufTarget[isTop][1] = 9999; // isTop ? 9999 : uncompressedTargetCount[isTop];
	totalCount += 2;
	badCount += 2;
	logV("** reset buffer, 2 bad frames dropped\n");
}


/*
u32 jdecInFunc(JDEC* jdec, BYTE* buff, UINT ndata) {
	u32 dataRead = ndata;
	if (dataRead > jdecRemainIn) {
		dataRead = jdecRemainIn;
	}
	if (buff) {
		memcpy(buff, jdecInputBuf, dataRead);
	}

	jdecInputBuf += dataRead;
	jdecRemainIn -= dataRead;
	return dataRead;
}

u32 jdecOutFunc(JDEC* jdec, void* buff, JRECT* rect) {
	u8* sp = (u8*) buff;

	for (int i = rect->top; i <= rect->bottom; i++) {
		for (int j = rect->left; j <= rect->right; j++) {
			//printf("offset: %d\n", (i * jdecWidth + j) * 3);
			u8* dp = jdecOutputBuf + (240 * i + j) * 3 ;
			dp[0] = sp[0];
			dp[1] = sp[1];
			dp[2] = sp[2];
			sp += 3;
		}
	}
	
	return 1;
}
*/

void uncompressJpeg(u8* src, u8* dst, u32 srcSize, u32 dstSize, u32 width, u32 height) {
	tjDecompress2(decompressHandle, src, srcSize, dst, width, 3 * width, height, TJPF_RGB, 0);
}

void drawFrame(int isTop, int addr) {
	if (isTop) {
		compressCount += 1;
		totalCompressRatio += ((float)bufTarget[isTop][addr]) / uncompressedTargetCount[isTop];
		uncompressJpeg(recvBuffer[isTop][addr], decompressBuffer[1], (PACKET_SIZE - 4) * bufTarget[isTop][addr], 400 * 240 * 3, 240, 400);

		convertBuffer(topBuffer, decompressBuffer[1], 400, 240, lastFormat);
		topRequireUpdate = 1;
	}
	else {

		compressCount += 1;
		totalCompressRatio += ((float)bufTarget[isTop][addr]) / uncompressedTargetCount[isTop];
		uncompressJpeg(recvBuffer[isTop][addr], decompressBuffer[0], (PACKET_SIZE - 4) * bufTarget[isTop][addr], 400 * 240 * 3, 240, 320);

		convertBuffer(botBuffer, decompressBuffer[0], 320, 240, lastFormat);
		botRequireUpdate = 1;
	}
}

int recoverFrame(int isTop, int addr) {
	// never recover any frame
	return 0;

	if (isTop) {
		// never recover a compressed frame
		return 0;
	}
	int loss = bufTarget[isTop][addr] - bufCount[isTop][addr];
	if (loss <= 10) {
		recoverCount += 1;
		logV("recovered one frame, usTop: %d, loss: %d\n",isTop,  loss);
		drawFrame(isTop, addr);
		return 1;
	}
	return 0;
}

int socketThreadMain(void* lpParameter)
{

	int ret;
	int serSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (serSocket == -1)
	{
		printf("socket error !");
		return 0;
	}

	struct sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(8001);
	serAddr.sin_addr.s_addr = INADDR_ANY;
	if (bind(serSocket, (struct sockaddr *)&serAddr, sizeof(serAddr)) == -1)
	{
		printf("bind error !");
		close(serSocket);
		return 0;
	}

	struct sockaddr_in remoteAddr;
	socklen_t nAddrLen = sizeof(remoteAddr);

	int lastRecvCount = 0;

	int buff_size = 8 * 1024 * 1024;
	socklen_t tmp = 4;

	ret = setsockopt(serSocket, SOL_SOCKET, SO_RCVBUF, (char*)(&buff_size), sizeof(buff_size));
	buff_size = 0;
	ret = getsockopt(serSocket, SOL_SOCKET, SO_RCVBUF, (char*)(&buff_size), &tmp);
	printf("set buff size: %d\n", buff_size);
	if (ret) {
		printf("set buff size failed, ret: %d\n", ret);
		return 0;
	}
	
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    ret = setsockopt(serSocket, SOL_SOCKET, SO_RCVTIMEO, (char *) &timeout,
            sizeof (timeout));
        if (ret) {
        printf("set recv timeout failed, ret: %d\n", ret);
        return 0;
    }

	resetBuffer(0);
	resetBuffer(1);
	
	while (1)
	{
		if (totalCount >= 100) {
			float quality = ((totalCount - badCount) * 1.0f + ( recoverCount) * 0.5f) / totalCount * 100;
			float compressRate = 0;
			float fps = 0;
			if (compressCount > 0) {
				compressRate = totalCompressRatio / compressCount;
			}
			logI("Quality: %.0f%% (total: %d, recover: %d, drop: %d)\n", quality,  totalCount, recoverCount,  badCount - recoverCount, compressRate);
			float timePassed = ((float)(clock() - lastTime) / CLOCKS_PER_SEC);
			if (timePassed > 0.1) {
				fps = compressCount / timePassed;
			}
			logI("fps: %.2f, compress rate: %.2f\n", fps , compressRate);
			lastTime = clock();
			totalCount = 0;
			badCount = 0;
			recoverCount = 0;
			compressCount = 0;
			totalCompressRatio = 0;

			char buf[512];
			sprintf(buf, "NTRViewer - Quality: %.0f%%, fps: %.2f", quality, fps);
			SDL_SetWindowTitle(mainWindow, buf);
		}
		int ret = recvfrom(serSocket, (char*)buf, 2000, 0, (struct sockaddr *)&remoteAddr, &nAddrLen);
		if (ret <= 0) {
			activateStreaming(activateStreamingHost);
			continue;
		}

		u8 id = buf[0];
		u8 isTop = buf[1] & 1;
		u8 format = buf[2];
		u8 cnt = buf[3];

		if (isTop > 1) {
			continue;
		}

		if (format != lastFormat) {
			logI("format changed: %d\n", format);
			lastFormat = format;

		}

		if (getIdDiff(id, trackingId[isTop]) == 2) {
			if (!isFinished[isTop][newestBuf[isTop]]) {
				if (!isFinished[isTop][!newestBuf[isTop]]) {
					// try to recover the frame before dropping it
					recoverFrame(isTop, !newestBuf[isTop]);
				}
			}
			// drop a frame
			advanceBuffer(isTop);
			trackingId[isTop] += 1;
		}
		else if (getIdDiff(id, trackingId[isTop]) > 2) {
			int shouldTryOlderOne = 0;
			if (!isFinished[isTop][newestBuf[isTop]]) {
				if (!recoverFrame(isTop, newestBuf[isTop])) {
					// failed to recover the newest frame, try older one
					shouldTryOlderOne = 1;
				}
			}
			if (shouldTryOlderOne) {
				if (!isFinished[isTop][!newestBuf[isTop]]) {
					recoverFrame(isTop, !newestBuf[isTop]);
				}
			}
			if (getIdDiff(trackingId[isTop], id) <= 3) {
				// maybe it is from very previous frame, ignore them
				logI("ignoring previous frame: %d\n", id);
				continue;
			}
			// drop all pending frames
			resetBuffer(isTop);
			trackingId[isTop] = id;
		}
		u32 offsetInBuffer = (u32)(cnt)* (PACKET_SIZE - 4);
		
		int bufAddr = 0;
		if (id == trackingId[isTop]) {
			bufAddr = !newestBuf[isTop];

		} else {
			bufAddr = newestBuf[isTop];

		}
		if (buf[1] & 0x10) {
			// the last compressed packet
			bufTarget[isTop][bufAddr] = cnt + 1;
		}
		memcpy(recvBuffer[isTop][bufAddr] + offsetInBuffer, buf + 4, (PACKET_SIZE - 4));
		bufCount[isTop][bufAddr] += 1;

		if (bufCount[isTop][bufAddr] >= bufTarget[isTop][bufAddr]) {
			if (bufCount[isTop][bufAddr] > bufTarget[isTop][bufAddr]) {
				// we have receive same packet multiple times?
				printf("wow\n");
			}
			isFinished[isTop][bufAddr] = 1;
			if ((isFinished[isTop][newestBuf[isTop]]) && (bufAddr != newestBuf[isTop])) {
				// the newest frame has already finished, do not draw
				printf("newest frame already finished: %d %d\n", isTop, id);
			}
			
			logV("good frame: %d %d\n", isTop, id);
			if (isTop) {
				logV("compression rate: %.2f\n", (float)(bufTarget[isTop][bufAddr]) / uncompressedTargetCount[isTop]);
			}
			drawFrame(isTop, bufAddr);
		}

		

	}
	close(serSocket);
	return 0;
}


void mainLoop() {
	const float real_FPS = 1000/60;
	//SDL_Rect topRect = { 0, 0, 400, 240 };
	//SDL_Rect botRect = { 40, 240, 320, 240 };
	SDL_Rect topRect = { 0, 60, 1600, 960 };
	SDL_Rect botRect = { 1600, 420, 320, 240 };
	
	/*if (layoutMode == 0) {
		topRect = { 0, 0, 400 * topScaleFactor, 240 * topScaleFactor };
		botRect = { 400 * topScaleFactor, screenHeight - 240 * botScaleFactor, 320 * botScaleFactor, 240 * botScaleFactor };
	}
	else {
		topRect = { 0, 0, 400 * topScaleFactor, 240 * topScaleFactor };
		float indent = 400 * topScaleFactor - 320 * botScaleFactor;
		botRect = { indent / 2, 240*topScaleFactor, 320 * botScaleFactor, 240 * botScaleFactor };

	}*/

	
	
    SDL_Thread *thread;
    thread = SDL_CreateThread(socketThreadMain, "Superthread", (void *)NULL);

	SDL_RenderClear(renderer);

	SDL_Event e;
	uint8_t quit = 0;
	while (!quit){
		while (SDL_PollEvent(&e)){
			if (e.type == SDL_QUIT){
				quit = 1;
			}
		}
		
		Uint32 start;
		start = SDL_GetTicks();
		
		/*if (topRequireUpdate) {
			topRequireUpdate = 0;
			SDL_UpdateTexture(topTexture, NULL, topBuffer, 400  * 3);
			SDL_RenderCopy(renderer, topTexture, NULL, &topRect);
			SDL_RenderPresent(renderer);

		}

		if (botRequireUpdate) {
			botRequireUpdate = 0;
			SDL_UpdateTexture(botTexture, NULL, botBuffer, 320 * 3);
			SDL_RenderCopy(renderer, botTexture, NULL, &botRect);
			SDL_RenderPresent(renderer);

		}*/
		
		SDL_UpdateTexture(topTexture, NULL, topBuffer, 400 * 3);
		SDL_UpdateTexture(botTexture, NULL, botBuffer, 320 * 3);
		SDL_RenderCopy(renderer, topTexture, NULL, &topRect);
		SDL_RenderCopy(renderer, botTexture, NULL, &botRect);
		SDL_RenderPresent(renderer);
		
		//SDL_Delay(10);
		while(real_FPS > SDL_GetTicks()-start)
		{
			SDL_Delay(real_FPS-(SDL_GetTicks()-start));
		}
	}
}


void parseOpts(int argc, char* argv[]) {
	const char* optString = "l:t:b:";
	char opt = getopt(argc, argv, optString);
	while (opt != -1) {
		switch (opt) {
		case 'l':
			layoutMode = atoi(optarg);
			break;
		case 't':
			topScaleFactor = atof(optarg);
			break;
		case 'b':
			botScaleFactor = atof(optarg);
			break;
		default:
			break;
		}

		opt = getopt(argc, argv, optString);
	}
}

int buildPacket(u32* buf, u32 type, u32 cmd, u32 arg0, u32 arg1, u32 arg2, u32 arg3) {
    int packetSize = 84;
    memset((void*) buf, 0, packetSize);
    buf[0] = 0x12345678;
    buf[1] = 1;
    buf[2] = type;
    buf[3] = cmd;
    buf[4] = arg0;
    buf[5] = arg1;
    buf[6] = arg2;
    buf[7] = arg3;
    return packetSize;
}

void activateStreaming(char* host) {
    u32 buf[64];
    
    struct sockaddr_in client_addr;
    client_addr.sin_family = AF_INET;   
    client_addr.sin_addr.s_addr = htons(INADDR_ANY);
    client_addr.sin_port = htons(0);    
    
    int client_socket = socket(AF_INET,SOCK_STREAM,0);
    if( client_socket < 0)
    {
        printf("Create Socket Failed!\n");
        exit(1);
    }
    
    if( bind(client_socket,(struct sockaddr*)&client_addr,sizeof(client_addr)))
    {
        printf("Client Bind Port Failed!\n"); 
        exit(1);
    }
 
	struct sockaddr_in server_addr = { 0 };
    server_addr.sin_family = AF_INET;
#ifdef WIN32
	server_addr.sin_addr.s_addr = inet_addr(host);
#else
	int res = inet_aton(host, &server_addr.sin_addr);
	while(res != 1)
	{
		res = inet_aton(host, &server_addr.sin_addr);
		sleep(1);
	}
	/*if (res == 0)
	{
		printf("Server IP Address Error!\n");
		exit(1);
	}
	printf("res %d\n", res);*/
#endif
	

    server_addr.sin_port = htons(8000);
    socklen_t server_addr_length = sizeof(server_addr);
    res = connect(client_socket,(struct sockaddr*)&server_addr, server_addr_length);
    while (res < 0)
    {
		res = connect(client_socket,(struct sockaddr*)&server_addr, server_addr_length);
		sleep(1);
        //printf("Connect To %s failed!\n",host);
       // exit(1);
    }
    printf("connected. \n");
    int mode = 1;
    if (priorityMode) {
        mode = 0;
    }
    int qosInByte = (int) (qosValue * 1024 * 1024 / 8);
    int packetSize = buildPacket(buf, 0, 901,  (mode << 8) | priorityFactor, jpegQuality, qosInByte, 0);
    send(client_socket, (const char*) buf, packetSize, 0);
    sleep(1);
    // 5
    for (int i = 0; i < 5; i++ ){
        
        int packetSize = buildPacket(buf, 0, 0, 0, 0, 0, 0);
		if (send(client_socket, (const char*)buf, packetSize, 0) < 0) {
            printf("send ping packet failed!\n");
            exit(1);
        } else {
            printf("sent ping packet.\n");
        }
        sleep(1);
    }
	close(client_socket);
}


#undef main

int main(int argc, char* argv[])
{
	lastTime = clock();

	decompressHandle = tjInitDecompress();

	parseOpts(argc, argv);
	
	snprintf(activateStreamingHost, sizeof(activateStreamingHost), argv[1]);
	printf("IP : %s\n", activateStreamingHost);
	
	if (strlen(activateStreamingHost) > 0) {
        activateStreaming(activateStreamingHost);
    }

	if (SDL_Init(SDL_INIT_VIDEO) != 0) {
		return 0;
	}

	if (layoutMode == 0) {
		screenWidth = 400 * topScaleFactor + 320 * botScaleFactor;
		screenHeight = 240 * topScaleFactor;
	}
	else {
		screenWidth = 400 * topScaleFactor ;
		screenHeight = 240 * topScaleFactor + 240 * botScaleFactor;
	}
	mainWindow = SDL_CreateWindow("NTRViewer", 100, 100, 1920, 1080, 0);
	renderer = SDL_CreateRenderer(mainWindow, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	//SDL_RenderSetScale(renderer, topScaleFactor, topScaleFactor);
	if (!renderer) {
		return 0;
	}

	topTexture = SDL_CreateTexture(renderer,
		SDL_PIXELFORMAT_RGB24,
		SDL_TEXTUREACCESS_STREAMING,
		400, 240);

	botTexture = SDL_CreateTexture(renderer,
		SDL_PIXELFORMAT_RGB24,
		SDL_TEXTUREACCESS_STREAMING,
		320, 240);


	mainLoop();
	return 0;
}

