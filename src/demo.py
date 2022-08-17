#!/usr/bin/env python3

import os
import time
import rospy
import sqlite3
import subprocess
import numpy as np
from sqlite3 import Error
from millihex_robot import Millihexapod

def main():
    """
    Usage: $ rosrun millihexapod demo.py
    """

    try:
        # Initialize Millihex robot
        millihex = Millihexapod()

        # Start roscore
        subprocess.Popen('roscore')
        time.sleep(1)

        # Initialize rospy node
        rospy.init_node('robot_rock', anonymous=True)
        rospy.sleep(1)

        # Start Gazebo
        millihex.spawn_model("gazebo")

        # Gait and obstacle parameters, range=(min, max)
        x = np.pi/2             # (pi/8, pi/2)
        z = np.pi/2             # (pi/8, pi)
        h = 0.02                # (0.01, 0.15)
        step = 0.02             # (0.01, 0.5)
        stance = np.pi/2        # (pi/8, pi/2)
        pattern = "tripod"      # ["bipod", "tripod", "quadruped", "pentapod"]

        # Spawn models and start walk test
        millihex.spawn_model("obstacle", args=[f"obstacle_h:={h}"])
        millihex.spawn_model("millihex")
        (success, duration) = millihex.walk(pattern=pattern, gait_x=x, gait_z=z, stance=stance, step=step)

        # Insert parameters to results database
        insert_result(x, z, h, step, stance, pattern, success, duration)

        # Parameters (N^6 data points)
        N = 4
        x_range = np.linspace(np.pi/8, np.pi/2, N)
        z_range = np.linspace(np.pi/8, np.pi, N)
        h_range = np.linspace(0.01, 0.15, N)
        step_range = np.linspace(0.01, 0.5, N)
        stance_range = np.linspace(np.pi/8, np.pi/2, N)
        pattern_range = ["bipod", "tripod", "quadruped", "pentapod"]

        # Parameter sweep and data collection
        for pattern in pattern_range:
            for x in x_range:
                for z in z_range:
                    for h in h_range:
                        for step in step_range:
                            for stance in stance_range:
                                millihex.spawn_model("obstacle", args=[f"obstacle_h:={h}"])
                                millihex.spawn_model("millihex")
                                (success, duration) = millihex.walk(pattern=pattern, gait_x=x, gait_z=z, stance=stance, step=step)
                                insert_result(x, z, h, step, stance, pattern, success, duration)

    except rospy.ROSInterruptException:
        pass
    finally:
        print("Kill all processes...\n")
        os.system("killall -9 rosmaster & killall -9 gzserver & killall -9 gzclient")


def create_connection(db_file):
    """ create a database connection to the SQLite database
        specified by db_file
    :param db_file: database file
    :return: Connection object or None
    https://www.sqlitetutorial.net/sqlite-python/
    """
    conn = None
    try:
        conn = sqlite3.connect(db_file)
        return conn
    except Error as e:
        print(e)
    return conn


def create_results_table():
    # Database file path for parameter sweep results table
    database = r"/root/catkin_ws/src/millihexapod/db/results.db"

    # Results table SQL command
    results_table = """ CREATE TABLE IF NOT EXISTS results (
                            x REAL NOT NULL,
                            z REAL NOT NULL,
                            h REAL NOT NULL,
                            step REAL NOT NULL,
                            stance REAL NOT NULL,
                            pattern TEXT NOT NULL,
                            success INTEGER NOT NULL,
                            duration INTEGER NOT NULL
                        ); """

    # Create a database connection
    conn = create_connection(database)
    if conn is not None:
        cur = conn.cursor()
        cur.execute(results_table)
    else:
        print("Error! Could not create the database connection.")


def insert_result(x, z, h, step, stance, pattern, success, duration):
    # Database file path for parameter sweep results table
    database = r"/root/catkin_ws/src/millihexapod/db/results.db"

    # Insert row SQL command
    row = """ INSERT INTO results(x, z, h, step, stance, pattern, success, duration)
              VALUES(?, ?, ?, ?, ?, ?, ?, ?) """
    result = (x, z, h, step, stance, pattern, success, duration)

    # Create a database connection
    conn = create_connection(database)
    with conn:
        cur = conn.cursor()
        cur.execute(row, result)
        conn.commit()


if __name__ == '__main__':
    # create_results_table()
    main()