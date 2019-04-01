#!/usr/bin/env python3

import logging

import grpc
import time
## import go generated files
import festo_pb2
import festo_pb2_grpc


def run():

        ## grpc channel to localhost:50051, to be connected with running valve binary.
    with grpc.insecure_channel("localhost:50051") as channel:
        ## initialize Client
        client = festo_pb2_grpc.valveControllerStub(channel)
        ## funtions we have openValve and closeValve
        client.openValve(festo_pb2.ValveRequest(valveNo=2))
        time.sleep(0.5)
        client.closeValve(festo_pb2.ValveRequest(valveNo=2))
        ## we offer Valve 1, 2 or both (4)





if __name__ == '__main__':
    logging.basicConfig()
    run()
