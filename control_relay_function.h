
bool toggleState(bool state){
  Serial.println("");
  Serial.print(state);
  state = !state;
  Serial.print("  Toggle to ");
  Serial.println(state);
  return state;
}


bool check_lastTimes(long currentTimes , long lastTimes){
  if( currentTimes - lastTimes < 0){
    return true;
  }
  else{ 
    return false;
  }
}

long set_lastTimes(long currentTimes , long lastTimes){
  if(check_lastTimes(currentTimes, lastTimes)){
    return 0;
  }
  else{ 
    return lastTimes;
  }
}