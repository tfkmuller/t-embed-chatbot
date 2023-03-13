# t-embed-chatbot
leveraging the lilygo t-embed hardware to create a voice chatbot with chatgpt. 
This code doesnt work yet...
It is based on the example code from lilygo for the t-embed board and on the chatgpt library from "That Project"

The idea is to leverage the T-Embed to do the following:
1.) record audio/speech from the 2 microphones when you hold the rotary button onto the SD card
2.) send the audio file to Google speech2text API or other STT service
3.) save returned text onto SD card
4.) send text file to openai chatGPT API
5.) save returned answer to SD card
6.) send text file to Google text2speech API or Whisper.AI
7.) save returned audio file to SD card
8.) display answer on TFT (scroll through text with rotary dial) and at the same time output audio file via built in I2S speaker
