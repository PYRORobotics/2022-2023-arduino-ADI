#!/bin/bash
cd src/proto
python ../../nanopb/generator/nanopb_generator.py -I ../../proto  ../../proto/messages.proto