#!/bin/bash

git st
git add .
git ci -m "$1"
git push origin master
