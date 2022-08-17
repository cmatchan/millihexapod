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
        stance = np.pi/2        # (pi/8, pi/2)
        step = 0.02             # (0.01, 0.5)
        h = 0.02                # (0.01, 0.15)
        pattern = "tripod"      # ["bipod", "tripod", "quadruped", "pentapod"]

        # Spawn models and start walk test
        millihex.spawn_model("obstacle", args=[f"obstacle_h:={h}"])
        millihex.spawn_model("millihex")
        millihex.walk(pattern=pattern, gait_x=x, gait_z=z, stance=stance, step=step)


        # Parameter sweep (N^5 data points)
        N = 4
        x_range = np.linspace(np.pi/8, np.pi/2, N)
        z_range = np.linspace(np.pi/8, np.pi, N)
        stance_range = np.linspace(np.pi/8, np.pi/2, N)
        step_range = np.linspace(0.01, 0.5, N)
        h_range = np.linspace(0.01, 0.15, N)
        pattern_range = ["bipod", "tripod", "quadruped", "pentapod"]

        # Data collection
        for x in x_range:
            for z in z_range:
                for stance in stance_range:
                    for step in step_range:
                        for h in h_range:
                            for pattern in pattern_range:
                                millihex.spawn_model("obstacle", args=[f"obstacle_h:={h}"])
                                millihex.spawn_model("millihex")
                                millihex.walk(pattern, gait_x=x, gait_z=z, stance=stance, step=step)

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


def create_table(conn, create_table_sql):
    """ create a table from the create_table_sql statement
    :param conn: Connection object
    :param create_table_sql: a CREATE TABLE statement
    :return:
    https://www.sqlitetutorial.net/sqlite-python/
    """
    try:
        c = conn.cursor()
        c.execute(create_table_sql)
    except Error as e:
        print(e)


def create_project(conn, project):
    """
    Create a new project into the projects table
    :param conn:
    :param project:
    :return: project id
    """
    sql = """ INSERT INTO projects(name,begin_date,end_date)
              VALUES(?,?,?) """
    cur = conn.cursor()
    cur.execute(sql, project)
    conn.commit()


def init_tables():
    # Database file path
    database = r"/root/catkin_ws/src/millihexapod/db/pythonsqlite.db"

    # Create new tables
    sql_create_projects_table = """ CREATE TABLE IF NOT EXISTS projects (
                                        id integer PRIMARY KEY,
                                        name text NOT NULL,
                                        begin_date text,
                                        end_date text
                                    ); """

    sql_create_tasks_table = """CREATE TABLE IF NOT EXISTS tasks (
                                    id integer PRIMARY KEY,
                                    name text NOT NULL,
                                    priority integer,
                                    status_id integer NOT NULL,
                                    project_id integer NOT NULL,
                                    begin_date text NOT NULL,
                                    end_date text NOT NULL,
                                    FOREIGN KEY (project_id) REFERENCES projects (id)
                                ); """

    # create a database connection
    conn = create_connection(database)

    # create tables
    if conn is not None:
        # create projects table
        create_table(conn, sql_create_projects_table)

        # create tasks table
        create_table(conn, sql_create_tasks_table)
    else:
        print("Error! Could not create the database connection.")


def table_insert():
    # Database file path
    database = r"/root/catkin_ws/src/millihexapod/db/pythonsqlite.db"

    # create a database connection
    conn = create_connection(database)
    with conn:
        # create a new project
        project = ('Cool App with SQLite & Python', '2015-01-01', '2015-01-30');
        create_project(conn, project)


if __name__ == '__main__':
    # init_tables()
    table_insert()
    # main()