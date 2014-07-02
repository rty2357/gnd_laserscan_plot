/*
 * gnd_laserscan_model_node.cpp
 *
 *  Created on: 2014/06/23
 *      Author: tyamada
 */

#include <stdio.h>
#include <math.h>
#include <gnd/gnd-util.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "gnd/rosmsgs/laserscan_reader.hpp"


int main( int argc, char *argv[] ) {
	{ // ---> init ROS
		ros::init(argc, argv, "gnd_laserscan_plot_node");
	} // <--- init ROS

	ros::NodeHandle nodehandle;							// node handle
	ros::Subscriber subsc_laserscan;					// laser scan message subscriber

	gnd::rosmsgs::LaserScanReader reader_laserscan;		// laser scan topic reader


	{ // ---> initialize
		int phase = 0;

		fprintf(stdout, "1. init ROS\n");
		fprintf(stdout, "2. init laserscan data publish\n");
		fprintf(stdout, "\n");


		{ // ---> init laserscan data subscribe
			phase++;
			fprintf(stdout, "%d. init laserscan data publish\n", phase);

			// allocate buffer
			reader_laserscan.allocate(100);

			// subscribe
			subsc_laserscan = nodehandle.subscribe("scan", 200,
					&gnd::rosmsgs::LaserScanReader::rosmsg_read,
					reader_laserscan.reader_pointer() );

			fprintf(stdout, "    ... OK\n\n");
		} // <--- init laserscan data subscribe


	} // <--- initialize



	{ // ---> operate
		// it's a test for read laser scan message 2014/06/23
		FILE *pp;
		sensor_msgs::LaserScan scan;
		ros::Rate loop_rate(10);

		// gnuplot pipe open
		pp = popen("gnuplot -persist","w");
		fprintf(pp, "set size ratio -1\n");
		fprintf(pp, "set palette model RGB function gray,0,0\n");
		fprintf(pp, "unset colorbox\n");
		fprintf(pp, "set xr [-5:5]\n");
		fprintf(pp, "set yr [-5:5]\n");
		fflush(pp);

		while( ros::ok() ) {
			FILE* fp;
			int cnt = 0;
			loop_rate.sleep();

			ros::spinOnce();

			// get latest scan
			if( reader_laserscan.copy_latest(&scan) < 0 )	continue;


			fp = fopen("plot.dat", "w");
			for( int i = 0; i < scan.ranges.size(); i++ ) {
				double x, y;
				double angle;
				angle = scan.angle_min + scan.angle_increment * i;

				x = scan.ranges[i] * cos(angle + gnd_deg2rad(90));
				y = scan.ranges[i] * sin(angle + gnd_deg2rad(90));

				if( scan.intensities.size() > i ) {
					fprintf(fp, "%lf %lf %lf\n", x, y, scan.intensities[i]);
				}
				else {
					fprintf(fp, "%lf %lf\n", x, y );
				}
				cnt++;
			}
			fprintf(fp, "\n");
			fclose(fp);
			if( cnt ) {
				if( scan.intensities.size() <= 0 ) {
					fprintf(pp, "plot \"plot.dat\" w p u 1:2\n");
				}
				else {
					static const int level = 10;
					static const double min = 1000;
					static const double max = 6000;

					fprintf(pp, "plot \"./plot.dat\" u 1:($3 < %lf ? $2 : 1/0) pt 7 pointsize 0.3 lc palette frac %lf notitle,", min + max / level, 0.0);
					for( int i = 2; i < level - 1; i++ ) {
						fprintf(pp, "\"./plot.dat\" u 1:($3 < %lf ? 1/0 : $3 < %lf ? 1/0 : $2) pt 7 pointsize 0.3 lc palette frac %lf notitle,", min + (i - 1) * max / level, i * max / level, (double)i / level);
					}
					fprintf(pp, "\"./plot.dat\" u 1:($3 > %lf ?  $2 : 1/0) pt 7 pointsize 0.3  lc palette frac %lf notitle\n", (level - 1) * max / level, 1.0);
				}
				fflush(pp);
			}
		}

		fprintf(pp, "quit\n");
		fflush(pp);
		pclose(pp);
	} // <--- operate



	{ // ---> finalize

	} // <--- finalize

	return 0;
}
