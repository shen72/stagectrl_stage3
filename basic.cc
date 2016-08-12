// Arkapravo Bhaumik (arkapravobhaumik at gmail dot com) 
// 23 January 2012
// This is a modification of Dr.Vaughan's work available at http://github.com/rtv/stagectrl
// Modified to work on Stage 3.X.X, Tested on Stage 3.2.2, Ubuntu 10.04 LTS

#include <stage.hh>
#include <vector>

using namespace Stg;

const double cruisespeed = 0.4; //initial value was 0.4
const double avoidspeed = 0.05; //initial value was 0.05
const double avoidturn = 0.5; //initial value was 0.5
const double minfrontdistance = 1.0; // 0.6  
const bool verbose = true;
const double stopdist = 0.3; 
const int avoidduration = 10;

struct robot_t
{
  ModelPosition* pos;
  ModelRanger* ranger;
  int avoidcount, randcount;
};

int LaserUpdate( Model* mod, robot_t* robot );
int PositionUpdate( Model* mod, robot_t* robot );

// Stage calls this when the model starts up
extern "C" int Init( Model* mod, CtrlArgs* args )
{
  // local arguments
	/*  printf( "\nWander controller initialised with:\n"
			"\tworldfile string \"%s\"\n" 
			"\tcmdline string \"%s\"",
			args->worldfile.c_str(),
			args->cmdline.c_str() );
	*/
  puts("Init.");

  robot_t* robot = new robot_t;
 
  robot->avoidcount = 0;
  robot->randcount = 0;
  
  robot->pos = (ModelPosition*)mod;
  robot->ranger = (ModelRanger*)mod->GetChild( "ranger:0" );
  robot->ranger->AddCallback(
      Model::CB_UPDATE, (model_callback_t)LaserUpdate, robot );
  
  robot->ranger->Subscribe(); // starts the laser updates
  robot->pos->Subscribe(); // starts the position updates
    
  return 0; //ok
}


// inspect the laser data and decide what to do
int LaserUpdate( Model* mod, robot_t* robot )
{
  // Only look at the sensor 0.
  const auto& sensors = robot->ranger->GetSensors();
  if (sensors.size() == 0) {
    puts("sensor not found.");
    return 0;
  }
  const auto& sensor = sensors[0]; 
  // get the data
  uint32_t sample_count = sensor.ranges.size();
 
  bool obstruction = false;
  bool stop = false;

  // find the closest distance to the left and right and check if
  // there's anything in front
  double minleft = 1e6;
  double minright = 1e6;

  for (uint32_t i = 0; i < sample_count; i++)
    {

		if( verbose ) printf( "%.3f ", sensor.ranges[i] );

      if( (i > (sample_count/3)) 
			 && (i < (sample_count - (sample_count/3))) 
			 && sensor.ranges[i] < minfrontdistance)
		  {
			 if( verbose ) puts( "  obstruction!" );
			 obstruction = true;
		  }
		
      if( sensor.ranges[i] < stopdist )
		  {
			 if( verbose ) puts( "  stopping!" );
			 stop = true;
		  }
      
      if( i > sample_count/2 )
				minleft = std::min( minleft, sensor.ranges[i] );
      else      
				minright = std::min( minright, sensor.ranges[i] );
    }
  
  if( verbose ) 
	 {
		puts( "" );
		printf( "minleft %.3f \n", minleft );
		printf( "minright %.3f\n ", minright );
	 }

  if( obstruction || stop || (robot->avoidcount>0) )
    {
      if( verbose ) printf( "Avoid %d\n", robot->avoidcount );
	  		
      robot->pos->SetXSpeed( stop ? 0.0 : avoidspeed );      
      
      /* once we start avoiding, select a turn direction and stick
	 with it for a few iterations */
      if( robot->avoidcount < 1 )
        {
			 if( verbose ) puts( "Avoid START" );
          robot->avoidcount = random() % avoidduration + avoidduration;
			 
			 if( minleft < minright  )
				{
				  robot->pos->SetTurnSpeed( -avoidturn );
				  if( verbose ) printf( "turning right %.2f\n", -avoidturn );
				}
			 else
				{
				  robot->pos->SetTurnSpeed( +avoidturn );
				  if( verbose ) printf( "turning left %2f\n", +avoidturn );
				}
        }
		
      robot->avoidcount--;
    }
  else
    {
      if( verbose ) puts( "Cruise" );

      robot->avoidcount = 0;
      robot->pos->SetXSpeed( cruisespeed );	  
      robot->pos->SetTurnSpeed(  0 );
    }

 //  if( robot->pos->Stalled() )
// 	 {
// 		robot->pos->SetSpeed( 0,0,0 );
// 		robot->pos->SetTurnSpeed( 0 );
// 	 }
    
  return 0;
}

int PositionUpdate( Model* mod, robot_t* robot )
{
  Pose pose = robot->pos->GetPose();

  printf( "Pose: [%.2f %.2f %.2f %.2f]\n",
	  pose.x, pose.y, pose.z, pose.a );

  return 0; // run again
}

