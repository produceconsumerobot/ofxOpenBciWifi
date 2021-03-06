#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

	// ** Setup OpenBCI **
	openBci.setTcpPort(3000);
	openBci.enableDataLogging(ofToDataPath(ofGetTimestampString("%Y-%m-%d-%H-%M-%S") + ".log"));

	// ** Setup oscilloscopes **
	nHeadsets = 2;
	nChan = 8;
	int Fs = 250;

	ofTrueTypeFont legendFont;
	legendFont.load("verdana.ttf", 7, true, true);

	std::vector<ofColor> oscColors = {
		ofColor(255, 0, 0),
		ofColor(196, 196, 196),
		ofColor(255, 255, 0),
		ofColor(128, 128, 128),
		ofColor(255, 0, 255),
		ofColor(255, 140, 0),
		ofColor(196, 0, 196),
		ofColor(139, 69, 19)
	};

	std::vector<string> oscNames = { "ch1", "ch2", "ch3", "ch4", "ch5", "ch6", "ch7", "ch8" };

	// Data scope setup
	float timeWindow = 5.f; // seconds
	float yScale = 0.004f; // yScale multiplier
	float yOffset = 0.f; // yOffset from the center of the scope window
	ofPoint winSize = ofGetWindowSize();
	yTop = 100;
	xGap = 20;
	scopeWins.resize(nHeadsets);
	ofRectangle scopeArea;
	scopeArea = ofRectangle(ofPoint(0, yTop), ofPoint(winSize.x / 2 * 3 / 4 - xGap, winSize.y));
	scopeWins.at(0) = ofxMultiScope(nChan, scopeArea, legendFont, 40); // Setup the multiScope panel
	scopeArea = ofRectangle(ofPoint(winSize.x / 2 + xGap, yTop), ofPoint(winSize.x / 2 + winSize.x / 2 * 3 / 4, winSize.y));
	scopeWins.at(1) = ofxMultiScope(nChan, scopeArea, legendFont, 40); // Setup the multiScope panel
	for (int h = 0; h < nHeadsets; h++)
	{
		for (int ch = 0; ch < oscColors.size(); ch++)
		{
			scopeWins.at(h).scopes.at(ch).setup(timeWindow, Fs, oscNames.at(ch), oscColors.at(ch), yScale, yOffset); // Setup each oscilloscope panel
		}
	}

	// FFT scope setup
	nFftBins = openBci.getFftBinFromFrequency(60);
	scopeFftWins.resize(nHeadsets);
	scopeArea = ofRectangle(ofPoint(winSize.x / 2 * 3 / 4 - xGap, yTop), ofPoint(winSize.x / 2 - xGap, winSize.y));
	scopeFftWins.at(0) = ofxMultiScope(nChan, scopeArea, legendFont, -1); // Setup the multiScope panel
	scopeArea = ofRectangle(ofPoint(winSize.x / 2 + winSize.x / 2 * 3 / 4, yTop), ofPoint(winSize.x, winSize.y));
	scopeFftWins.at(1) = ofxMultiScope(nChan, scopeArea, legendFont, -1); // Setup the multiScope panel
	for (int h = 0; h < nHeadsets; h++)
	{
		for (int ch = 0; ch < oscColors.size(); ch++)
		{
			scopeFftWins.at(h).scopes.at(ch).setup(1, nFftBins, oscNames.at(ch), oscColors.at(ch), 40, -1200); // Setup each oscilloscope panel
		}
	}

	lastFftMillis.resize(nHeadsets);
	fftDelay.resize(nHeadsets);
	for (int i = 0; i < lastFftMillis.size(); i++)
	{
		lastFftMillis.at(i) = ofGetElapsedTimeMillis();
	}

	isPaused = false;
	selectedScope = 0;

	// Debug Logger
	debugLogger.setDirPath("");
	debugLogger.setFilename(ofToDataPath(ofGetTimestampString("%Y-%m-%d-%H-%M-%S") + ".debug_log"));
	debugLoggingEnabled = false;
	debugLogger.startThread();
}

//--------------------------------------------------------------
void ofApp::update(){
	ofBackground(0, 0, 0);

	openBci.update();

	stringData = openBci.getStringData();

	vector<string> ipAddresses = openBci.getHeadsetIpAddresses();
	for (int h = 0; h < ipAddresses.size() && h < nHeadsets; h++)
	{
		if (debugLoggingEnabled)
		{
			debugLogger.push(stringData.at(h) + "\n");
		}

		vector<vector<float>> tempData = openBci.getData(ipAddresses.at(h));
		vector<vector<float>> tempFftData;
		if (openBci.isFftNew(ipAddresses.at(h)))
		{
			tempFftData = openBci.getLatestFft(ipAddresses.at(h));

			fftDelay.at(h) = ofGetElapsedTimeMillis() - lastFftMillis.at(h);
			lastFftMillis.at(h) = ofGetElapsedTimeMillis();
		}

		for (int ch = 0; ch < tempData.size() && ch < nChan; ch++)
		{
			if (!isPaused)
			{
				scopeWins.at(h).scopes.at(ch).updateData(tempData.at(ch));

				if (openBci.isFftNew(ipAddresses.at(h)))
				{
					tempFftData.at(ch).resize(nFftBins);
					scopeFftWins.at(h).scopes.at(ch).updateData(tempFftData.at(ch));
				}
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetHexColor(0xDDDDDD);
	ofDrawBitmapString("ofxOpenBciWifi Example\nconnect on port: " + ofToString(openBci.getTcpPort()), 10, 20);

	ofDrawBitmapString("Frame Rate (Hz): " + ofToString(ofGetFrameRate()) + 
		"\nFFT Buffer Fill Rates (Hz): " + ofToString(1000.f / fftDelay.at(0)) + ", " + ofToString(1000.f / fftDelay.at(1)), ofGetWindowSize().x / 2, 20);
	
	int yDraw = yTop - 7;
	vector<string> ipAddresses = openBci.getHeadsetIpAddresses();
	string ipString = "";
	if (ipAddresses.size() > 0) ipString = ipAddresses.at(0);
	ofDrawBitmapString("OpenBCI Headset 1: " + (ipAddresses.size() > 0 ? ipAddresses.at(0) : ""), 10, yDraw);
	if (ipAddresses.size() > 1) ipString = ipAddresses.at(1);
	ofDrawBitmapString("OpenBCI Headset 2: " + (ipAddresses.size() > 1 ? ipAddresses.at(1) : ""), ofGetWindowSize().x / 2 + xGap + 10, yDraw);
	ofDrawBitmapString("FFT", ofGetWindowSize().x / 2 * 3 / 4 - xGap, yDraw);
	ofDrawBitmapString("FFT", ofGetWindowSize().x / 2 + ofGetWindowSize().x / 2 * 3 / 4 + xGap, yDraw);
	for (int h = 0; h < nHeadsets; h++)
	{
		if (ipAddresses.size() > h)
		{
			string stringData = openBci.getStringData(ipAddresses.at(h));
			ofDrawBitmapString(ipAddresses.at(h) + " JSON: " + stringData, 10, 50 + 15 * h);
		}
		// OpenBCI Headset Data: ip
		// FFT
		scopeWins.at(h).plot();
		scopeFftWins.at(h).plot();
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
	if (key == ' ')
	{
		isPaused = !isPaused;
	}

	// Increment the yScale
	if (key == '+') {
		if (selectedScope == 0) {
			for (int h = 0; h < nHeadsets; h++)
			{
				scopeWins.at(h).incrementYScale();
			}
		}
	}

	// Decrement the yScale
	if ((key == '-') || (key == '_')) {
		if (selectedScope == 0) {
			for (int h = 0; h < nHeadsets; h++)
			{
				scopeWins.at(h).decrementYScale();
			}
		}
	}

	// Increment the yOffset
	if (key == 357) { // Up Arrow
		if (selectedScope == 0) {
			for (int h = 0; h < nHeadsets; h++)
			{
				scopeWins.at(h).incrementYOffset();
			}
		}
	}

	// Decrement the yOffset
	if (key == 359) { // Down Arrow
		if (selectedScope == 0) {
			for (int h = 0; h < nHeadsets; h++)
			{
				scopeWins.at(h).decrementYOffset();
			}
		}
	}

	// Increment the timeWindow
	if (key == 358) { // Right Arrow
		if (selectedScope == 0) {
			for (int h = 0; h < nHeadsets; h++)
			{
				scopeWins.at(h).incrementTimeWindow();
			}
		}
	}

	// Decrement the timeWindow
	if (key == 356) { // Left Arrow
		if (selectedScope == 0) {
			for (int h = 0; h < nHeadsets; h++)
			{
				scopeWins.at(h).decrementTimeWindow();
			}
		}
	}
	if (key == '0') {
		selectedScope = 0;
	}
	if (key == '1') {
		selectedScope = 1;
	}
	if (key == '2') {
		selectedScope = 2;
	}

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
