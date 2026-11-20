# Engine-control-software
Three software packages are provided here: 1) software (EngineModule_DataGram_2025_1126_1334.ino) operating on an Adafruit Feather M0 with RFM69HCW 433MHz Packet Radio (https://www.adafruit.com/product/3177),  2) ("conduit") software also running on an Adafruit Feather M0 with RFM69HCW 433MHz Packet Radio (https://www.adafruit.com/product/3177) that is connected to a control computer via USB and communicates with the engine module over 433 MHz packet radio and  3) software that runs on a controller computer that allows an operator to select engines and control their speed, direction, lights, etc.

Note the engine module is written in Arduino (C++ - like code), as is the "conduit" software that sits between the controller computer and the engine module.  The top-level controlling software is written in Python.  Packet radio libraries that came from Radiohead (https://www.airspayce.com/mikem/arduino/RadioHead/) work best with the compiled Arduino code over the interpeted Python code that can also run on the Adafruit Feathers.  I used the Radiohead reliable datagrams feature to greatly increase the odds that a sent packet was received in what can be an electronically noisy model train environment.  

I chose 433MHz radios because of their range, and their ability to penetrate obstructions.  They have worked reliably with my setup in a 51' by 17' 6" room.  Note that standard wireless radios could also be used in many environments, which could gain the advantage of guaranteed message delivery (TCP rather than UDP), and lower cost.  

While my code is heavily documented internally, I still could have done better.  My apologies if you have to work a little harder to figure things out.

An article on this effort appears in the January 2027 issue of Model Railroader Magazine (https://www.trains.com/mrr/).  It's strongly recommended you see the article.

