#!/bin/bash

rosnode list | grep record* | xargs rosnode kill