//////////////////////////
////Required Libraries////
//////////////////////////


////////////////
////Settings////
////////////////


////////////////////////////
////Class Instantiations////
////////////////////////////


/////////////
////Setup////
/////////////

String message;

void setup()
{
  Serial.begin(115200);
}

/////////////////
////Main Loop////
/////////////////

void loop() {
  if(Serial.available()) {
    char c = Serial.read();

    if(c == ',' || c == '\n') {
      parse();
      message = "";
    }
    else if(c != -1) {
      message += c;
    }
  }
}

void parse() {
}
