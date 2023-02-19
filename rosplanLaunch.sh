#!/bin/bash

echo Generating problem...
rosservice call /rosplan_problem_interface/problem_generation_server

echo Planning...
rosservice call /rosplan_planner_interface/planning_server

echo Parsing...
rosservice call /rosplan_parsing_interface/parse_plan

echo Dispatching...
rosservice call /rosplan_plan_dispatcher/dispatch_plan

