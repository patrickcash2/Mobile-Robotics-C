// Edited: Patrick Cash, Keaton Rodgers
// Lab 7 - Wall Following
// ECEN345
// based on provided skeleton code

/*
libcapi324v22x
libprintf_flt
libm
*/

#include "capi324v221.h"

// ---------------------- Defines:

#define DEG_90  140     /* Number of steps for a 90-degree (in place) turn. */
#define DEG_180 280

// Desc: This macro-function can be used to reset a motor-action structure
//       easily.  It is a helper macro-function.
#define __RESET_ACTION( motor_action )    \
do {                                  \
	( motor_action ).speed_L = 0;         \
	( motor_action ).speed_R = 0;         \
	( motor_action ).accel_L = 0;         \
	( motor_action ).accel_R = 0;         \
	( motor_action ).state = STARTUP;     \
	} while( 0 ) /* end __RESET_ACTION() */

	// Desc: This macro-fuction translates action to motion -- it is a helper
	//       macro-function.
	#define __MOTOR_ACTION( motor_action )   \
	do {                                 \
		STEPPER_set_accel2( ( motor_action ).accel_L, ( motor_action ).accel_R ); \
		STEPPER_runn( ( motor_action ).speed_L, ( motor_action ).speed_R ); \
		} while( 0 ) /* end __MOTOR_ACTION() */

		// Desc: This macro-function is used to set the action, in a more natural
		//       manner (as if it was a function).

		// ---------------------- Type Declarations:

		// Desc: The following custom enumerated type can be used to specify the
		//       current state of the robot.  This parameter can be expanded upon
		//       as complexity grows without intefering with the 'act()' function.
		//		 It is a new type which can take the values of 0, 1, or 2 using
		//		 the SYMBOLIC representations of STARTUP, EXPLORING, etc.
		typedef enum ROBOT_STATE_TYPE {

			STARTUP = 0,    // 'Startup' state -- initial state upon RESET.
			EXPLORING,      // 'Exploring' state -- the robot is 'roaming around'.
			AVOIDING,        // 'Avoiding' state -- the robot is avoiding a collision.
			HOMING

		} ROBOT_STATE;

		// Desc: Structure encapsulates a 'motor' action. It contains parameters that
		//       controls the motors 'down the line' with information depicting the
		//       current state of the robot.  The 'state' variable is useful to
		//       'print' information on the LCD based on the current 'state', for
		//       example.
		typedef struct MOTOR_ACTION_TYPE {

			ROBOT_STATE state;              // Holds the current STATE of the robot.
			signed short int speed_L;       // SPEED for LEFT  motor.
			signed short int speed_R;       // SPEED for RIGHT motor.
			unsigned short int accel_L;     // ACCELERATION for LEFT  motor.
			unsigned short int accel_R;     // ACCELERATION for RIGHT motor.
			
		} MOTOR_ACTION;

		// Desc: Structure encapsulates 'sensed' data.  Right now that only consists
		//       of the state of the left & right IR sensors when queried.  You can
		//       expand this structure and add additional custom fields as needed.
		typedef struct SENSOR_DATA_TYPE {

			BOOL left_IR;       // Holds the state of the left IR.
			BOOL right_IR;      // Holds the state of the right IR.

			// from photo resistors
			float left_PH;
			float right_PH;
			
			// from sonar
			unsigned long int dist_SONAR;
			unsigned long int prev_dist_SONAR;
		} SENSOR_DATA;

		// ---------------------- Globals:
		volatile MOTOR_ACTION action;  	// This variable holds parameters that determine
		// the current action that is taking place.
		// Here, a structure named "action" of type
		// MOTOR_ACTION is declared.

		// ---------------------- Prototypes:
		void IR_sense( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms );
		void PHOTO_sense( volatile SENSOR_DATA *pSensors );
		void SONAR_sense( volatile SENSOR_DATA *pSensors );
		void explore( volatile MOTOR_ACTION *pAction );
		void light_follow( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensor );
		void wall_follow( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensor, int right );
		void IR_avoid( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors );
		void SONAR_avoid( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors );
		void act( volatile MOTOR_ACTION *pAction );
		void info_display( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensor );
		BOOL compare_actions( volatile MOTOR_ACTION *a, volatile MOTOR_ACTION *b );

		// ---------------------- Convenience Functions:
		void info_display( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensor )
		{

			// NOTE:  We keep track of the 'previous' state to prevent the LCD
			//        display from being needlessly written, if there's  nothing
			//        new to display.  Otherwise, the screen will 'flicker' from
			//        too many writes.
			static ROBOT_STATE previous_state = STARTUP;

			if ( ( pAction->state != previous_state ) || ( pAction->state == STARTUP ) )
			{

				LCD_clear();

				//  Display information based on the current 'ROBOT STATE'.
				switch( pAction->state )
				{

					case STARTUP:

					// Fill me in.

					break;

					case EXPLORING:

					LCD_printf( "Exploring...\n" );

					break;

					case AVOIDING:

					LCD_printf( "Avoiding...\n" );

					break;
					
					case HOMING:
					
					LCD_printf( "Homing...\n" );
					
					break;

					default:

					LCD_printf( "Unknown state!\n" );

				} // end switch()

				// Note the new state in effect.
				previous_state = pAction->state;

			} // end if()

		} // end info_display()

		// ----------------------------------------------------- //
		BOOL compare_actions( volatile MOTOR_ACTION *a, volatile MOTOR_ACTION *b )
		{

			// NOTE:  The 'sole' purpose of this function is to
			//        compare the 'elements' of MOTOR_ACTION structures
			//        'a' and 'b' and see if 'any' differ.

			// Assume these actions are equal.
			BOOL rval = TRUE;

			if ( ( a->state   != b->state )   ||
			( a->speed_L != b->speed_L ) ||
			( a->speed_R != b->speed_R ) ||
			( a->accel_L != b->accel_L ) ||
			( a->accel_R != b->accel_R ) )

			rval = FALSE;

			// Return comparison result.
			return rval;

		} // end compare_actions()

		// ---------------------- Top-Level Behaviorals:
		void SONAR_sense( volatile SENSOR_DATA *pSensors )
		{
			unsigned long int	usonic_time_us;
			SWTIME				usonic_time_ticks;
			float				distance_cm; // Holds the distance-to-target in 'cm'.
			
			usonic_time_ticks = USONIC_ping();
			usonic_time_us = 10 * (( unsigned long int )( usonic_time_ticks ));
			
			pSensors->dist_SONAR = 0.01724 * usonic_time_us;
		}
		void PHOTO_sense( volatile SENSOR_DATA *pSensors )
		{
			ADC_SAMPLE sample;
			
			ADC_set_channel( ADC_CHAN6 );
			sample = ADC_sample();
			pSensors->left_PH = ( ( sample * 5.0f ) / 1024 );
			ADC_set_channel( ADC_CHAN7 );
			sample = ADC_sample();
			pSensors->right_PH = ( ( sample * 5.0f ) / 1024 );

		} // end photo sense()
		void IR_sense( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms )
		{

			// Sense must know if it's already sensing.
			//
			// NOTE: 'BOOL' is a custom data type offered by the CEENBoT API.
			//
			static BOOL timer_started = FALSE;
			
			// The 'sense' timer is used to control how often gathering sensor
			// data takes place.  The pace at which this happens needs to be
			// controlled.  So we're forced to use TIMER OBJECTS along with the
			// TIMER SERVICE.  It must be 'static' because the timer object must remain
			// 'alive' even when it is out of scope -- otherwise the program will crash.
			static TIMEROBJ sense_timer;
			
			// If this is the FIRST time that sense() is running, we need to start the
			// sense timer.  We do this ONLY ONCE!
			if ( timer_started == FALSE )
			{
				
				// Start the 'sense timer' to tick on every 'interval_ms'.
				//
				// NOTE:  You can adjust the delay value to suit your needs.
				//
				TMRSRVC_new( &sense_timer, TMRFLG_NOTIFY_FLAG, TMRTCM_RESTART,
				interval_ms );
				
				// Mark that the timer has already been started.
				timer_started = TRUE;
				
			} // end if()
			
			// Otherwise, just do the usual thing and just 'sense'.
			else
			{

				// Only read the sensors when it is time to do so (e.g., every
				// 125ms).  Otherwise, do nothing.
				if ( TIMER_ALARM( sense_timer ) )
				{

					// NOTE: Just as a 'debugging' feature, let's also toggle the green LED
					//       to know that this is working for sure.  The LED will only
					//       toggle when 'it's time'.
					LED_toggle( LED_Green );

					// Read the left and right sensors, and store this
					// data in the 'SENSOR_DATA' structure.
					pSensors->left_IR  = ATTINY_get_IR_state( ATTINY_IR_LEFT  );
					pSensors->right_IR = ATTINY_get_IR_state( ATTINY_IR_RIGHT );

					// NOTE: You can add more stuff to 'sense' here.
					
					// Snooze the alarm so it can trigger again.
					TIMER_SNOOZE( sense_timer );
					
				} // end if()

			} // end else.

		} // end sense()
		// -------------------------------------------- //
		void explore( volatile MOTOR_ACTION *pAction )
		{
			
			// Nothing to do, but set the parameters to explore.  'act()' will do
			// the rest down the line.
			pAction->state = EXPLORING;
			
			pAction->speed_L = 200;
			pAction->speed_R = 200;
			pAction->accel_L = 400;
			pAction->accel_R = 400;
			
			// That's it -- let 'act()' do the rest.
			
		} // end explore()
		void wall_follow( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensor, int right )
		{
				unsigned long int WALL_DIST = 25;		// ~10 INCHES
				unsigned long int MAX_WALL_DIST = 60;
/*				long int desired_dist = 25;
				
				pAction->speed_L = 200 - (desired_dist - pSensor->dist_SONAR) + (pSensor->prev_dist_SONAR - (desired_dist - pSensor->dist_SONAR));
				pAction->speed_R = 200 + (desired_dist - pSensor->dist_SONAR) + (pSensor->prev_dist_SONAR - (desired_dist - pSensor->dist_SONAR));
				pAction->accel_L = 400;
				pAction->accel_R = 400;
				
				pSensor->prev_dist_SONAR = pSensor->dist_SONAR;*/

				/*
				else if (actual_dist > MAX_WALL_DIST)
				{
					Kp = 100;
					pAction->speed_L = Tp - turn;
					pAction->speed_R = Tp + turn;
					pAction->accel_L = 400;
					pAction->accel_R = 400;
					
					pSensor->prev_dist_SONAR = actual_dist;
				}
				
				*/
				
				if (right)
				{
					// follow right wall
					if ( pSensor->dist_SONAR > (WALL_DIST) && pSensor->dist_SONAR < MAX_WALL_DIST )
					{
						pAction->state = HOMING;
					
						// turn right
						pAction->speed_R = 175;
						pAction->speed_L = 200;
						pAction->accel_L = 400;
						pAction->accel_R = 400;		
					}
					// sharp right follow
					else if ( pSensor->dist_SONAR > MAX_WALL_DIST )
					{
						pAction->state = HOMING;
					
						// turn right
						pAction->speed_R = 75;
						pAction->speed_L = 200;
						pAction->accel_L = 400;
						pAction->accel_R = 400;
					}
					// veer left
					else if ( pSensor->dist_SONAR < (WALL_DIST) )
					{
						pAction->state = HOMING;
					
						// turn left
						pAction->speed_R = 200;
						pAction->speed_L = 100;
						pAction->accel_L = 400;
						pAction->accel_R = 400;		
					}
				}
				else
				{
					// follow left wall
					if ( pSensor->dist_SONAR > (WALL_DIST) && pSensor->dist_SONAR < MAX_WALL_DIST )
					{
						pAction->state = HOMING;
						
						// turn left
						pAction->speed_R = 200;
						pAction->speed_L = 175;
						pAction->accel_L = 400;
						pAction->accel_R = 400;
					}
					// sharp left follow
					else if ( pSensor->dist_SONAR > MAX_WALL_DIST )
					{
						pAction->state = HOMING;
						
						// turn left
						pAction->speed_R = 200;
						pAction->speed_L = 75;
						pAction->accel_L = 400;
						pAction->accel_R = 400;
					}
					// veer right
					else if ( pSensor->dist_SONAR < (WALL_DIST) )
					{
						pAction->state = HOMING;
						
						// turn right
						pAction->speed_R = 100;
						pAction->speed_L = 200;
						pAction->accel_L = 400;
						pAction->accel_R = 400;
					}
				}
				

		}
		void light_follow( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensor )
		{
			// a ballistic reaction to being right in front of a light
			if ( (pSensor->left_PH + pSensor->right_PH) > 8 )
			{
				// Note that we're avoiding...
				pAction->state = AVOIDING;

				// STOP!
				STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_OFF );
				
				// Back up...
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_REV, 300, 200, 400, STEPPER_BRK_OFF,
				STEPPER_REV, 300, 200, 400, STEPPER_BRK_OFF );
				
				// ... and turn RIGHT ~90-deg.
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_FWD, DEG_180, 200, 400, STEPPER_BRK_OFF,
				STEPPER_REV, DEG_180, 200, 400, STEPPER_BRK_OFF );

				// ... and set the motor action structure with variables to move forward.

				pAction->state = EXPLORING;
				pAction->speed_L = 200;
				pAction->speed_R = 200;
				pAction->accel_L = 400;
				pAction->accel_R = 400;
			}
			else if ( (pSensor->left_PH + pSensor->right_PH) > 4 )	// WARNING: this may be too high for dark room
			{
				pAction->state = HOMING;
				
				pAction->speed_R = 100 * ( pSensor->left_PH );
				pAction->speed_L = 100 * ( pSensor->right_PH );
				pAction->accel_L = 400;
				pAction->accel_R = 400;
			}
		}
		// -------------------------------------------- //
		void SONAR_avoid( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors )
		{
			unsigned long int MAX_THRESHOLD = 40;
			unsigned long int MIN_THRESHOLD = 9;
			unsigned long int GAIN = 12;
			// needs a sense
			// sense sends out a signal then immediately goes to input
			// sonar pings and sends back a signal if object in range
			// receiving signal can trigger response, pulse width
			// is how to determine distance from object
			// 1/2 to account for pulse being sensor to target
			// and back and we only want sensor to target
			// d = (1/2) * 34480 cm/s * t => d = 17240 * t
			// if we take input as us, then d = 0.017240 * t
			
			if ( pSensors->dist_SONAR > MIN_THRESHOLD && pSensors->dist_SONAR < MAX_THRESHOLD )
			{
				pAction->state = AVOIDING;
				
				pAction->speed_R = 200 + GAIN * ( MAX_THRESHOLD - pSensors->dist_SONAR );
				pAction->speed_L = 200;
				pAction->accel_L = 400;
				pAction->accel_R = 400;
			}
			
			// signal pin for USONIC is PA3 which is p1 on J3
		}
		// -------------------------------------------- //
		void IR_avoid( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors )
		{

			// NOTE: Here we have NO CHOICE, but to do this 'ballistically'.
			//       **NOTHING** else can happen while we're 'avoiding'.
			
			// Example of ONE case (you can expand on this idea):
			
			// If the LEFT sensor tripped...
			if( pSensors->left_IR == TRUE )
			{

				// Note that we're avoiding...
				pAction->state = AVOIDING;

				// STOP!
				STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_OFF );
				
				// Back up...
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_REV, 150, 200, 400, STEPPER_BRK_OFF,
				STEPPER_REV, 150, 200, 400, STEPPER_BRK_OFF );
				
				// ... and turn RIGHT ~90-deg.
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_FWD, DEG_90, 200, 400, STEPPER_BRK_OFF,
				STEPPER_REV, DEG_90, 200, 400, STEPPER_BRK_OFF );

				// ... and set the motor action structure with variables to move forward.

				pAction->state = AVOIDING;
				pAction->speed_L = 200;
				pAction->speed_R = 200;
				pAction->accel_L = 400;
				pAction->accel_R = 400;
				
			} // end if()
			
		} // end avoid()
		// -------------------------------------------- //
		void act( volatile MOTOR_ACTION *pAction )
		{

			// 'act()' always keeps track of the PREVIOUS action to determine
			// if a new action must be executed, and to execute such action ONLY
			// if any parameters in the 'MOTOR_ACTION' structure have changed.
			// This is necessary to prevent motor 'jitter'.
			static MOTOR_ACTION previous_action = {

				STARTUP, 0, 0, 0, 0

			};

			if( compare_actions( pAction, &previous_action ) == FALSE )
			{

				// Perform the action.  Just call the 'free-running' version
				// of stepper move function and feed these same parameters.
				__MOTOR_ACTION( *pAction );

				// Save the previous action.
				previous_action = *pAction;

			} // end if()
			
		} // end act()
		// ---------------------- CBOT Main:
		void CBOT_main( void )
		{
			volatile SENSOR_DATA sensor_data;
			
			// ** Open the needed modules.
			LED_open();     // Open the LED subsystem module.
			LCD_open();     // Open the LCD subsystem module.
			STEPPER_open(); // Open the STEPPER subsystem module.
			ADC_open();		// Open the ADC for PHOTO-sensing.
			//STOPWATCH_open(); // We're not using this
			USONIC_open();	// Open the USONIC module for sonar avoidance.
			
			ADC_set_VREF( ADC_VREF_AVCC ); // 5V
			
			// Reset the current motor action.
			__RESET_ACTION( action );
			
			// Nofify program is about to start.
			LCD_printf( "Starting...\n" );
			
			// Wait 3 seconds or so.
			TMRSRVC_delay( TMR_SECS( 3 ) );
			
			// Clear the screen and enter the arbitration loop.
			LCD_clear();
			
			// TESTING WARNING MAY NEED TO REMOVE LATER
			ATTINY_set_RC_servo( RC_SERVO0, 850 );	// all the way right
			//ATTINY_set_RC_servo( RC_SERVO0, 2100 ); // all the way left
			
			// Enter the 'arbitration' while() loop -- it is important that NONE
			// of the behavior functions listed in the arbitration loop BLOCK!
			// Behaviors are listed in increasing order of priority, with the last
			// behavior having the greatest priority (because it has the last 'say'
			// regarding motor action (or any action)).
			while( 1 )
			{
				// Sense must always happen first.
				// (IR sense happens every 125ms).
				IR_sense( &sensor_data, 125 );
				
				//photo resistor sense
				// PHOTO_sense( &sensor_data );
				
				//sonar sensing
				SONAR_sense( &sensor_data );
				
				// Behaviors.
				explore( &action );
				
				// follows wall (0 means follow left, 1 means follow right)
				wall_follow( &action, &sensor_data, 1 );
				
				// homes on light
				// light_follow( &action, &sensor_data );
				
				// sonar avoiding
				// SONAR_avoid( &action, &sensor_data );
				
				// Note that 'avoidance' relies on sensor data to determine
				// whether or not 'avoidance' is necessary.
				IR_avoid( &action, &sensor_data );
				
				// Perform the action of highest priority.
				act( &action );

				// Real-time display info, should happen last, if possible (
				// except for 'ballistic' behaviors).  Technically this is sort of
				// 'optional' as it does not constitute a 'behavior'.
				info_display( &action, &sensor_data );
				
			} // end while()
			
		} // end CBOT_main()

