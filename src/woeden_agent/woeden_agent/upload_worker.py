#! /usr/bin/env python3

import os
import rclpy

from datetime import datetime
from interfaces.msg import UploadBytes
from interfaces.srv import Upload, UploadComplete
from rclpy.node import Node
from stream_zip import stream_zip, ZIP_64
from time import sleep

class WoedenUploadWorker(Node):

    def __init__(self):
        super().__init__("woeden_upload_worker")
        self.srv = self.create_service(Upload, "/upload_bag", self.service_callback)
        self.pub = self.create_publisher(UploadBytes, "/upload_chunks", 10)
        self.client = self.create_client(UploadComplete, '/upload_complete')
        self.timer = self.create_timer(10, self.timer_callback)
        self.is_uploading = False
        self.queue = []
    
    def service_callback(self, request, response):
        self.queue.append(request)
        response.success = True
        return response
    
    def timer_callback(self):
        if self.is_uploading or len(self.queue) == 0:
            return
        self.upload(self.queue.pop(0))
    
    def upload(self, request):
        self.is_uploading = True

        i = 0
        chunk_size = int(request.max_bandwidth)
        for chunk in stream_zip(self.unzipped_files(request.base_path, request.bag_uuid), chunk_size=chunk_size):
            msg = UploadBytes()
            msg.contents = chunk
            msg.index = i
            msg.bag_uuid = request.bag_uuid
            self.pub.publish(msg)

            i += 1
            sleep(1)
        
        self.req = UploadComplete.Request()
        self.req.bag_uuid = request.bag_uuid
        self.req.chunks = i

        self.client.call_async(self.req)        
        self.is_uploading = False

    def unzipped_files(self, base_path, bag_uuid):
        modified_at = datetime.now()
        perms = 0o600
        path = f'{base_path}/woeden/bags/{bag_uuid}'

        def get_bytes(file):
            with open(f'{path}/{file}', 'rb') as f:
                yield f.read()

        for file in os.listdir(path):
            yield file, modified_at, perms, ZIP_64, get_bytes(file)

def main(args=None):
    rclpy.init(args=args)
    worker = WoedenUploadWorker()
    rclpy.spin(worker)
    worker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
