//
// AE483GroundStation
// David Hanley
// PotentialField function written by Samyak Shah for Final Project, lines 157-422
// planner.c
// This file contains all functions for planning. The most basic example
// would be to hold position at some desired (x,y,z,yaw).
//

/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/

/*--------------- Includes ---------------*/

//system
#include <math.h>
#include <stdio.h>      
#include <stdlib.h>

#define bool int
#define false 0
#define true (!false)

//planner
#include "planner.h"

//Parameters
float param_katt = 30;
float param_batt = 0.5;
float param_krep = 0.35;
float param_brep = 0.35;
float param_kdescent = 1;
float param_bdescent = 0.5;

//Drone position and radius
float drone_q[3], drone_r = 0.2;

//Obstacle position and radius
float obst_q[9], obst_s = 0.2;

//Goal position
float goal_q[9];

//Vector from goal position to drone position
float v[3], v_norm, v_norm1, v_norm2, v_norm3;

//Vector from obstacle position to drone position
float vo[3], vo_norm;

//d and dgrad between drone and obstacle
float d, dgrad[3];

//Attractive, Repulsive and Total gradients
float gradfatt[3], gradfrep[3], gradf[3], gradf_norm;

//Gradient Descent
float dq[3], dqnorm;

// num targets
const int n = 3;
const int a = 2;
//Target relative positions
float x[3], y[3];
float target_dist[3], target_dist2[2];
float weight[3];
float score[3], score2[2];
float sort_td[3], sort_score[3];
float sort_td2[2], sort_score2[2];
float closest_target_dist, closest_target_dist2;
int target_order[3], target_order2[2];
float x2[3], y2[3];
float copy_x[3], copy_y[3], copy_z[3];



/*------------- End Includes -------------*/

/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/*----------------------------- Functions ------------------------------*/
/*----------------------------------------------------------------------*/
/*--------------- Quat2Euler/Euler2Quat ---------------*/
void Planner_Quat2Euler_f(Quat_f *quat_param, Euler_f *euler_param)
{
	//variables
	float sqw = quat_param->qw*quat_param->qw;
	float sqx = quat_param->qx*quat_param->qx;
	float sqy = quat_param->qy*quat_param->qy;
	float sqz = quat_param->qz*quat_param->qz;
	float unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor

	//singularity tests
	float test = quat_param->qx*quat_param->qy + quat_param->qz*quat_param->qw;
	if (test > 0.499*unit) { // singularity at north pole
		euler_param->ty = 2 * atan2f(quat_param->qx, quat_param->qw);
		euler_param->tz = PI_f / 2;
		euler_param->tx = 0;
		return;
	}
	if (test < -0.499*unit) { // singularity at south pole
		euler_param->ty = -2 * atan2f(quat_param->qx, quat_param->qw);
		euler_param->tz = -PI_f / 2;
		euler_param->tx = 0;
		return;
	}

	//no singularity
	euler_param->ty = atan2f(2 * quat_param->qy*quat_param->qw - 2 * quat_param->qx*quat_param->qz, sqx - sqy - sqz + sqw);
	euler_param->tz = asinf(2 * test / unit);
	euler_param->tx = atan2f(2 * quat_param->qx*quat_param->qw - 2 * quat_param->qy*quat_param->qz, -sqx + sqy - sqz + sqw);
}
/*------------- End Quat2Euler/Euler2Quat -------------*/

/*----- MakeState -----*/
void Planner_MakeState6f_PosQuat(State6_f *state_param, Pos_f *pos_param, Quat_f *quat_param)
{
	state_param->Pos.x = pos_param->x;
	state_param->Pos.y = pos_param->y;
	state_param->Pos.z = pos_param->z;
	Planner_Quat2Euler_f(quat_param, &(state_param->Ori));
}
/*--- End MakeState ---*/
/*-- Potential Field --*/

int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}

int switchcount = 0;
float prev_time = 0, delta_t;

int i, j, k;
//int plan_path = 1;
//int first_time = 0;
//int second_time = 0;
//int third_time = 0;
//int temp[3];


int PotentialField(State6_f state_param,State6_f ObstCurrent,State6_f ObstCurrent2, State6_f ObstCurrent3, PosYaw_f *dGoal, float time, int PotFlag, int plan_path, int first_time, int second_time, int third_time)
{
	if(PotFlag == 1)
	/*---------- Simple Desired Position Set ------------------*/
	{
		drone_q[0] = state_param.Pos.x;		drone_q[1] = state_param.Pos.y;		drone_q[2] = state_param.Pos.z;
		if(plan_path == 1) // run only one time
		{
			// get quad position
			drone_q[0] = state_param.Pos.x;		drone_q[1] = state_param.Pos.y;		drone_q[2] = state_param.Pos.z;

			// get target position
			obst_q[0] = ObstCurrent.Pos.x;		obst_q[1] = ObstCurrent.Pos.y;		obst_q[2] = ObstCurrent.Pos.z;
			obst_q[3] = ObstCurrent2.Pos.x;		obst_q[4] = ObstCurrent2.Pos.y;		obst_q[5] = ObstCurrent2.Pos.z;
			obst_q[6] = ObstCurrent3.Pos.x;		obst_q[7] = ObstCurrent3.Pos.y;		obst_q[8] = ObstCurrent3.Pos.z;

			// copy
			copy_x[0] = ObstCurrent.Pos.x;
			copy_y[0] = ObstCurrent.Pos.y;
			copy_z[0] = ObstCurrent.Pos.z;

			copy_x[1] = ObstCurrent2.Pos.x;
			copy_y[1] = ObstCurrent2.Pos.y;
			copy_z[1] = ObstCurrent2.Pos.z;

			copy_x[2] = ObstCurrent3.Pos.x;
			copy_y[2] = ObstCurrent3.Pos.y;
			copy_z[2] = ObstCurrent3.Pos.z;

			// assign weights to n = 3 targets (maybe a printf statement...)
			weight[0] = 10;
			weight[1] = 10;
			weight[2] = 10;

			// compute differece in component positions
			x[0] =  drone_q[0]-obst_q[0];
			y[0] =  drone_q[1]-obst_q[1];
			
			x[1] =  drone_q[0]-obst_q[3];
			y[1] =  drone_q[1]-obst_q[4];

			x[2] =  drone_q[0]-obst_q[6];
			y[2] =  drone_q[1]-obst_q[7];

			for(i = 0; i<n; i++)
			{
				// distance formula, compute absolute difference
				target_dist[i] = sqrt(x[i]*x[i]+y[i]*y[i]);
			}

			// compute closest target distance, n = 3 targets
			// make copy
			sort_td[0] = target_dist[0];
			sort_td[1] = target_dist[1];
			sort_td[2] = target_dist[2];

			// sort in ascending order
			qsort(sort_td, n, sizeof(int), compare);
			closest_target_dist = sort_td[0];

			for(i = 0; i<n; i++)
			{
				// scoring formula: score = x*weight-y*normalized distance, where x+y = 1
				score[i] = 0.4*weight[i]-0.6*target_dist[i]/closest_target_dist;
			}

			// rank scores (ascending order) to determine path (will need to adjust for n!=3 targets, loops are used sparsely to speed debugging)
			sort_score[0] = score[0];
			sort_score[1] = score[1];
			sort_score[2] = score[2];
			qsort(sort_score, n, sizeof(int), compare);

			target_order[0] = 9;
			target_order[1] = 9;
			target_order[2] = 9;

			if(score[0] == sort_score[0] && score[1] == sort_score[1] && score[2] == sort_score[2])
			{
				target_order[0] = 2;
				target_order[1] = 1;
				target_order[2] = 0;
			}
			else if(score[0] == sort_score[1] && score[1] == sort_score[0] && score[2] == sort_score[2])
			{
				target_order[0] = 1;
				target_order[1] = 2;
				target_order[2] = 0;
			}
			else if(score[0] == sort_score[2] && score[1] == sort_score[0] && score[2] == sort_score[1])
			{
				target_order[0] = 0;
				target_order[1] = 2;
				target_order[2] = 1;
			}
			else if(score[0] == sort_score[0] && score[1] == sort_score[2] && score[2] == sort_score[1])
			{
				target_order[0] = 2;
				target_order[1] = 0;
				target_order[2] = 1;
			}
			else if(score[0] == sort_score[1] && score[1] == sort_score[2] && score[2] == sort_score[0])
			{
				target_order[0] = 1;
				target_order[1] = 0;
				target_order[2] = 2;
			}
			else 
			{
				target_order[0] = 0;
				target_order[1] = 1;
				target_order[2] = 2;
			}

			// get first goal
			goal_q[0] = copy_x[target_order[0]];
			goal_q[1] = copy_y[target_order[0]];
			goal_q[2] = -1;
			
			// recompute scores relative to first goal -------------------------------

			x2[0] =  goal_q[0]-obst_q[3];
			y2[0] =  goal_q[1]-obst_q[4];
			
			x2[1] =  goal_q[0]-obst_q[6];
			y2[1] =  goal_q[1]-obst_q[7];
			

			for(i = 0; i<a; i++)
			{
				// distance formula
				target_dist2[i] = sqrt(x2[i]*x2[i]+y2[i]*y2[i]);
			}

			// compute closest target distance, a = 2 targets
			sort_td2[0] = target_dist2[0];
			sort_td2[1] = target_dist2[1];

			qsort(sort_td2, a, sizeof(int), compare);
			closest_target_dist2 = sort_td2[0];

			for(i = 0; i<a; i++)
			{
				// scoring formula: score = x*weight-y*normalized distance, where x+y = 1
				score2[i] = 0.4*weight[i]-0.6*target_dist2[i]/closest_target_dist2;
			}

			// rank scores (ascending order) to determine path
			sort_score2[0] = score2[0];
			sort_score2[1] = score2[1];

			qsort(sort_score2, a, sizeof(int), compare);

			// find target order: target_order[0] = first target, target_order[1] = second target, ...

			target_order2[0] = 9;
			target_order2[1] = 9;

			if(score2[0] == sort_score2[0] && score2[1] == sort_score2[1])
			{
				target_order2[0] = 1;
				target_order2[1] = 0;
			}
			else 
			{
				target_order2[0] = 0;
				target_order2[1] = 1;
			}

			// update second and third goal

			if(target_order[0] == 0)
			{
				copy_x[0] = copy_x[1];
				copy_x[1] = copy_x[2];

				copy_y[0] = copy_y[1];
				copy_y[1] = copy_y[2];
			}
			else if(target_order[0] == 1)
			{
				copy_x[0] = copy_x[0];
				copy_x[1] = copy_x[2];

				copy_y[0] = copy_y[0];
				copy_y[1] = copy_y[2];
			}

			else if(target_order[0] == 2)
			{
				copy_x[0] = copy_x[0];
				copy_x[1] = copy_x[1];

				copy_y[0] = copy_y[0];
				copy_y[1] = copy_y[1];
			}

			goal_q[3] = copy_x[target_order2[0]];
			goal_q[4] = copy_y[target_order2[0]];
			goal_q[5] = -1;

			goal_q[6] = copy_x[target_order2[1]];
			goal_q[7] = copy_y[target_order2[1]];
			goal_q[8] = -1;

			return 1; // path has been planned, go to target 1
		}
		
		delta_t = time - prev_time;

		printf("//////////////////////////////////////////\n");
		printf(" Targets: \n %0.4f  %0.4f  %0.4f\n %0.4f  %0.4f  %0.4f\n %0.4f  %0.4f  %0.4f\n \r", copy_x[0], copy_y[0], copy_z[0], copy_x[1], copy_y[1], copy_z[1], copy_x[2], copy_y[2], copy_z[2]);
		printf(" Target Order: \n %d  %d  %d \n \r", target_order[0], target_order[1], target_order[2]);
		printf(" Path Planned?: %d \n Going to Target: \n %d  %d  %d \n \r", plan_path, first_time, second_time, third_time);
		printf(" dGoal: %0.4f, %0.4f, %0.4f \n\r",dGoal->Pos.x, dGoal->Pos.y, dGoal->Pos.z);
		printf(" Goals: \n %0.4f  %0.4f  %0.4f\n %0.4f  %0.4f  %0.4f\n %0.4f  %0.4f  %0.4f\n \r", goal_q[0], goal_q[1], goal_q[2], goal_q[3], goal_q[4], goal_q[5], goal_q[6], goal_q[7], goal_q[8]);
		printf("//////////////////////////////////////////\n");

		if(first_time == 1)
		{
			v[0] = drone_q[0]-goal_q[0]; v[1] = drone_q[1]-goal_q[1];
			v_norm1 = sqrt(v[0]*v[0] + v[1]*v[1]);
			v_norm = v_norm1;

			dGoal->Pos.x = goal_q[0];
			dGoal->Pos.y = goal_q[1];
			dGoal->Pos.z = -1;
			dGoal->Tz = 0;
		}

		if(v_norm1 <= 0.3 && first_time == 1)
		{
			first_time = 0;
			second_time = 1;
			return 2; // go to target 2
		}

		if(second_time == 1)
		{
			v[0] = drone_q[0]-goal_q[3]; v[1] = drone_q[1]-goal_q[4];
			v_norm2 = sqrt(v[0]*v[0] + v[1]*v[1]);
			v_norm = v_norm2;

			dGoal->Pos.x = goal_q[3];
			dGoal->Pos.y = goal_q[4];
			dGoal->Pos.z = -1;
			dGoal->Tz = 0;
		}

		if (v_norm2 <= 0.3 && second_time == 1)
		{
			second_time = 0;
			third_time = 1;
			return 3; // go to target 3
		}

		if (third_time == 1)
		{
			v[0] = drone_q[0]-goal_q[6]; v[1] = drone_q[1]-goal_q[7]; 
			v_norm3 = sqrt(v[0]*v[0] + v[1]*v[1]);
			v_norm = v_norm3;

			dGoal->Pos.x = goal_q[6];
			dGoal->Pos.y = goal_q[7];
			dGoal->Pos.z = -1;
			dGoal->Tz = 0;
		}

		return 10;

		
	
		prev_time = time;
	}
	
	/*-------- End Simple Desired Position Set ----------------*/
}


/* End Potential Field */
/*----------------------------------------------------------------------*/
/*--------------------------- End Functions ----------------------------*/
/*----------------------------------------------------------------------*/