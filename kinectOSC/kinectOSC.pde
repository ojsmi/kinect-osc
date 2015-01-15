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
//PVector com = new PVector();                                   
//PVector com2d = new PVector();


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
      text( "BOUNDING BOX, USER " + userList[i] + ": ", 20, i * 60 );
      float[] bb = new float[9];
      //fromX, fromY, fromZ, toX, toY, toZ
      context.getBoundingBox( userList[i], bb );
      for( int j = 0; j < bb.length; j++ ){ 
        text( bb[j], 20 + (60 *j), i*60 + 30 );
      }
    }      
  }    
  
  
}

float[] kinectUserBoundstoWorldNormalised( float[] b ){
  float[] projective = new float[6];
  context.convertRealWorldToProjective( b, projective );
  float[] norm = {
    constrain( map( projective[0], 0, 640, 0, 1 ), 0, 1 ),
    constrain( map( projective[1], 0, 480, 0, 1 ), 0, 1 ),
    constrain( map( projective[2], 0, 10000, 0, 1 ), 0, 1 ), // this may not be correct...
    constrain( map( projective[3], 0, 640, 0, 1 ), 0, 1 ),
    constrain( map( projective[4], 0, 480, 0, 1 ), 0, 1 ),
    constrain( map( projective[5], 0, 10000, 0, 1 ), 0, 1 ) // this may not be correct...
  };
  return norm;
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

PVector kinectToBoundsNormalised( PVector v, float[] bounds  ){
  PVector projective = new PVector();
  context.convertRealWorldToProjective( v, projective );
  float normX = map( projective.x, bounds[0], bounds[3], 0, 1 );
  float normY = map( projective.y, bounds[1], bounds[4], 0, 1 );
  float normZ = map( projective.z, bounds[2], bounds[5], 0, 1 );
  normZ = constrain( normZ, 0, 1 );
  return new PVector( normX, normY, normZ );
}

JSONObject PVectorToJSONObject( PVector v ){
  JSONObject json = new JSONObject();
  try{
    json.setFloat( "x", v.x );
  } catch ( Exception e ){
    println( "caught exception on x: " + e.getMessage() + ", x = " + v.x );
    json.setFloat( "x", 0 );
  }
  try{
    json.setFloat( "y", v.y );
  } catch ( Exception e ){
    println( "caught exception on y: " + e.getMessage() + ", y = " + v.y );
    json.setFloat( "y", 0 );
  }
  try{
    json.setFloat( "z", v.z );
  } catch ( Exception e ){
    println( "caught exception on z: " + e.getMessage() + ", z = " + v.z );
    json.setFloat( "z", 0 );
  }
  return json;
}

JSONArray floatArrayToJSONArray( float[] a ){
  JSONArray json = new JSONArray();
  for( int i = 0; i < a.length; i++ ){
    try{
     json.setFloat( i, a[i] );
    } catch ( Exception e ){
      json.setFloat( i, 0 );
      println( "caught exception on float array item" + i + ": " + e.getMessage() + ", value: " + a[i] );
    }
  }
  return json;
}

void sendSkeleton(int userId)
{
  float[] bounds = new float[6];
  PVector centerOfMass = new PVector();

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
  
  PVector headUser = new PVector();
  PVector neckUser = new PVector();
  PVector rightShoulderUser = new PVector();
  PVector rightElbowUser = new PVector();
  PVector rightHandUser = new PVector();
  PVector leftShoulderUser = new PVector();
  PVector leftElbowUser = new PVector();
  PVector leftHandUser = new PVector();
  PVector torsoUser = new PVector();
  PVector rightHipUser = new PVector();
  PVector rightKneeUser = new PVector();
  PVector rightFootUser = new PVector();
  PVector leftHipUser = new PVector();
  PVector leftKneeUser = new PVector();
  PVector leftFootUser = new PVector();
  
  context.getBoundingBox( userId, bounds );
  context.getCoM( userId, centerOfMass );
  
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
  
  headUser = kinectToBoundsNormalised( head, bounds );
  neckUser = kinectToBoundsNormalised( neck, bounds );
  rightShoulderUser = kinectToBoundsNormalised( rightShoulder, bounds );
  rightElbowUser = kinectToBoundsNormalised( rightElbow, bounds );
  rightHandUser = kinectToBoundsNormalised( rightHand, bounds );
  leftShoulderUser = kinectToBoundsNormalised( leftShoulder, bounds );
  leftElbowUser = kinectToBoundsNormalised( leftElbow, bounds );
  leftHandUser = kinectToBoundsNormalised( leftHand, bounds );
  torsoUser = kinectToBoundsNormalised( torso, bounds );
  rightHipUser = kinectToBoundsNormalised( rightHip, bounds );
  rightKneeUser = kinectToBoundsNormalised( rightKnee, bounds );
  rightFootUser = kinectToBoundsNormalised( rightFoot, bounds );
  leftHipUser = kinectToBoundsNormalised( leftHip, bounds );
  leftKneeUser = kinectToBoundsNormalised( leftKnee, bounds );
  leftFootUser = kinectToBoundsNormalised( leftFoot, bounds );

  JSONObject skeletonNorm = new JSONObject();
  // this doesn't really mean much - the points are normalised to the bound, 
  // but it's here for compatibility with the regular message structure
  float[] boundsUser = { 0, 0, 0, 1, 1, 1 };
  skeletonNorm.setJSONArray( "bounds", floatArrayToJSONArray( boundsUser ) );
  skeletonNorm.setJSONObject( "head", PVectorToJSONObject( headUser ) );
  skeletonNorm.setJSONObject( "neck", PVectorToJSONObject( neckUser ) );
  skeletonNorm.setJSONObject( "rightShoulder", PVectorToJSONObject( rightShoulderUser ) );
  skeletonNorm.setJSONObject( "rightElbow", PVectorToJSONObject( rightElbowUser ) );
  skeletonNorm.setJSONObject( "rightHand", PVectorToJSONObject( rightHandUser ) );
  skeletonNorm.setJSONObject( "leftShoulder", PVectorToJSONObject( leftShoulderUser ) );
  skeletonNorm.setJSONObject( "leftElbow", PVectorToJSONObject( leftElbowUser ) );
  skeletonNorm.setJSONObject( "leftHand", PVectorToJSONObject( leftHandUser ) );
  skeletonNorm.setJSONObject( "torso", PVectorToJSONObject( torsoUser ) );
  skeletonNorm.setJSONObject( "rightHip", PVectorToJSONObject( rightHipUser ) );
  skeletonNorm.setJSONObject( "rightKnee", PVectorToJSONObject( rightKneeUser ) );
  skeletonNorm.setJSONObject( "rightFoot", PVectorToJSONObject( rightFootUser ) );
  skeletonNorm.setJSONObject( "leftHip", PVectorToJSONObject( leftHipUser ) );
  skeletonNorm.setJSONObject( "leftKnee", PVectorToJSONObject( leftKneeUser ) );
  skeletonNorm.setJSONObject( "leftFoot", PVectorToJSONObject( leftFootUser ) );

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
  float[] normalisedBounds = kinectUserBoundstoWorldNormalised( bounds );
  skeleton.setJSONArray( "bounds", floatArrayToJSONArray( normalisedBounds ) );
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
  
  OscMessage msgNorm = new OscMessage( "/user/" + userId + "/normalised" );
  msgNorm.add( skeletonNorm.toString() );
  oscP5.send( msgNorm, netAddress );

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

