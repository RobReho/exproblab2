#!/bin/bash

2> >(grep -v TF_REPEATED_DATA buffer_core)
