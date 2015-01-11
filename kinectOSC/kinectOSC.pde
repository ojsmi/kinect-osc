import SimpleOpenNI.*;
import oscP5.*;
import netP5.*;

OscP5 oscP5;
NetAddress netAddress;

SimpleOpenNI  context;
color[]       userClr = new color[]{ color(255,0,0),
                                     color(0,255,0),
                                     color(0,0,255),
                                     color(255,255,0),
                                     color(255,0,255),
                                     color(0,255,255)
                                   };
PVector com = new PVector();                                   
PVector com2d = new PVector();                                   

boolean DRAW = true;

void setup()
{
  size(640, 480);
  frameRate( 30 );
  /* start oscP5, listening for incoming messages at port 12000 */
  oscP5 = new OscP5( this, 12000 );
  //broadcasting on 12001
  netAddress = new NetAddress( "127.0.0.1", 12001 );
  context = new SimpleOpenNI(this);
  //mirror points
  context.setMirror( true );
  
  if(context.isInit() == false)
  {
     println("Can't init SimpleOpenNI, maybe the camera is not connected!"); 
     exit();
     return;  
  }
  
  // enable depthMap generation 
  context.enableDepth();

  // enable skeleton generation for all joints
  context.enableUser();
 
  background(200,0,0);

  stroke(0,0,255);
  strokeWeight(3);
  smooth();  

}

void draw()
{
  background( 0 );
  // update the cam
  context.update();
  if( DRAW ){ 
    image(context.depthImage(),0,0);
  }
  
  // draw the skeleton if it's available
  int[] userList = context.getUsers();
  //println( userList.length + " users");
  
  for( int i=0; i < userList.length; i++ ){
    if( context.isTrackingSkeleton( userList[i] ) ){
      if( DRAW ){
        stroke(userClr[ (userList[i] - 1) % userClr.length ] );
        drawSkeleton(userList[i]);
      }
      sendSkeleton( userList[i] );
    }      
  }    
}
PVector kinectToWorldNormalised( PVector v ){
  PVector projective = new PVector();
  context.convertRealWorldToProjective( v, projective );
  PVector norm = new PVector(
    map( projective.x, 0, 640, 0, 1 ),
    map( projective.y, 0, 480, 0, 1 ),
    map( projective.z, 0, 10000, 0, 1 ) // this may not be correct...
  );
  return norm;
}

PVector kinectToBodyNormalised( PVector v, PVector centerOfMass  ){
  PVector projective = new PVector();
  context.convertRealWorldToProjective( v, projective );
  PVector norm = new PVector(
    map( projective.x, 0, 640, 0, 1 ),
    map( projective.y, 0, 480, 0, 1 ),
    map( projective.z, 0, 10000, 0, 1 ) // this may not be correct...
  );
  return norm;
}

JSONObject PVectorToJSONObject( PVector v ){
  JSONObject json = new JSONObject();
  try{
    json.setFloat( "x", v.x );
  } catch ( Exception e ){
    println( "caught exception on x: " + e.getMessage() + ", x = " + v.x );
  }
  try{
    json.setFloat( "y", v.y );
  } catch ( Exception e ){
    println( "caught exception on y: " + e.getMessage() + ", y = " + v.y );
  }
  try{
    json.setFloat( "z", v.z );
  } catch ( Exception e ){
    println( "caught exception on z: " + e.getMessage() + ", z = " + v.z );
  }
  return json;
}

void sendSkeleton(int userId)
{
  //float[] bounds = new float[0];
  //PVector centerOfMass = new PVector();
  PVector head = new PVector();
  PVector neck = new PVector();
  PVector rightShoulder = new PVector();
  PVector rightElbow = new PVector();
  PVector rightHand = new PVector();
  PVector leftShoulder = new PVector();
  PVector leftElbow = new PVector();
  PVector leftHand = new PVector();
  PVector torso = new PVector();
  PVector rightHip = new PVector();
  PVector rightKnee = new PVector();
  PVector rightFoot = new PVector();
  PVector leftHip = new PVector();
  PVector leftKnee = new PVector();
  PVector leftFoot = new PVector();
  
  //context.getBoundingBox( userId, bounds );
 // printArray( bounds );
  //context.getCoM( userId, centerOfMass ); 
  
  context.getJointPositionSkeleton(userId, context.SKEL_HEAD, head);
  context.getJointPositionSkeleton(userId, context.SKEL_NECK, neck);
  context.getJointPositionSkeleton(userId, context.SKEL_RIGHT_SHOULDER, rightShoulder);
  context.getJointPositionSkeleton(userId, context.SKEL_RIGHT_ELBOW, rightElbow);
  context.getJointPositionSkeleton(userId, context.SKEL_RIGHT_HAND, rightHand);
  context.getJointPositionSkeleton(userId, context.SKEL_LEFT_SHOULDER, leftShoulder);
  context.getJointPositionSkeleton(userId, context.SKEL_LEFT_ELBOW, leftElbow);
  context.getJointPositionSkeleton(userId, context.SKEL_LEFT_HAND, leftHand);
  context.getJointPositionSkeleton(userId, context.SKEL_TORSO, torso);
  context.getJointPositionSkeleton(userId, context.SKEL_RIGHT_HIP, rightHip);
  context.getJointPositionSkeleton(userId, context.SKEL_RIGHT_KNEE, rightKnee);
  context.getJointPositionSkeleton(userId, context.SKEL_RIGHT_FOOT, rightFoot);
  context.getJointPositionSkeleton(userId, context.SKEL_LEFT_HIP, leftHip);
  context.getJointPositionSkeleton(userId, context.SKEL_LEFT_KNEE, leftKnee);
  context.getJointPositionSkeleton(userId, context.SKEL_LEFT_FOOT, leftFoot);
 
  head = kinectToWorldNormalised( head );
  neck = kinectToWorldNormalised( neck );
  rightShoulder = kinectToWorldNormalised( rightShoulder );
  rightElbow = kinectToWorldNormalised( rightElbow );
  rightHand = kinectToWorldNormalised( rightHand );
  leftShoulder = kinectToWorldNormalised( leftShoulder );
  leftElbow = kinectToWorldNormalised( leftElbow );
  leftHand = kinectToWorldNormalised( leftHand );
  torso = kinectToWorldNormalised( torso );
  rightHip = kinectToWorldNormalised( rightHip );
  rightKnee = kinectToWorldNormalised( rightKnee );
  rightFoot = kinectToWorldNormalised( rightFoot );
  leftHip = kinectToWorldNormalised( leftHip );
  leftKnee = kinectToWorldNormalised( leftKnee );
  leftFoot = kinectToWorldNormalised( leftFoot );
 
  JSONObject skeleton = new JSONObject();
  skeleton.setJSONObject( "head", PVectorToJSONObject( head ) );
  skeleton.setJSONObject( "neck", PVectorToJSONObject( neck ) );
  skeleton.setJSONObject( "rightShoulder", PVectorToJSONObject( rightShoulder ) );
  skeleton.setJSONObject( "rightElbow", PVectorToJSONObject( rightElbow ) );
  skeleton.setJSONObject( "rightHand", PVectorToJSONObject( rightHand ) );
  skeleton.setJSONObject( "leftShoulder", PVectorToJSONObject( leftShoulder ) );
  skeleton.setJSONObject( "leftElbow", PVectorToJSONObject( leftElbow ) );
  skeleton.setJSONObject( "leftHand", PVectorToJSONObject( leftHand ) );
  skeleton.setJSONObject( "torso", PVectorToJSONObject( torso ) );
  skeleton.setJSONObject( "rightHip", PVectorToJSONObject( rightHip ) );
  skeleton.setJSONObject( "rightKnee", PVectorToJSONObject( rightKnee ) );
  skeleton.setJSONObject( "rightFoot", PVectorToJSONObject( rightFoot ) );
  skeleton.setJSONObject( "leftHip", PVectorToJSONObject( leftHip ) );
  skeleton.setJSONObject( "leftKnee", PVectorToJSONObject( leftKnee ) );
  skeleton.setJSONObject( "leftFoot", PVectorToJSONObject( leftFoot ) );
  
  OscMessage msg = new OscMessage( "/user/" + userId + "/skeleton" );
  msg.add( skeleton.toString() );
  oscP5.send( msg, netAddress ); 
  
}



// draw the skeleton with the selected joints
void drawSkeleton(int userId)
{
  // to get the 3d joint data
  /*
  PVector jointPos = new PVector();
  context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_NECK,jointPos);
  println(jointPos);
  */
  
  context.drawLimb(userId, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK);

  context.drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND);

  context.drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND);

  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO);

  context.drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_HIP, SimpleOpenNI.SKEL_LEFT_KNEE);
  context.drawLimb(userId, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT);

  context.drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE);
  context.drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT);  
}

// -----------------------------------------------------------------
// SimpleOpenNI events

void onNewUser(SimpleOpenNI curContext, int userId)
{
  println("onNewUser - userId: " + userId);
  println("\tstart tracking skeleton");
  
  curContext.startTrackingSkeleton(userId);
  
  OscMessage msg = new OscMessage( "/userfound" );
  msg.add( userId );
  oscP5.send( msg, netAddress ); 
  
}

void onLostUser(SimpleOpenNI curContext, int userId)
{
  println("onLostUser - userId: " + userId);
  OscMessage msg = new OscMessage( "/userlost" );
  msg.add( userId );
  oscP5.send( msg, netAddress ); 
}

void onVisibleUser(SimpleOpenNI curContext, int userId)
{
  //println("onVisibleUser - userId: " + userId);
}


void keyPressed()
{
  switch(key)
  {
  case ' ':
    context.setMirror(!context.mirror());
    break;
  case 'd':
    DRAW = !DRAW;
    break;
  }
}  

