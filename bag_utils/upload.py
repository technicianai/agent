# import requests
# import shutil
# import sys


# args = sys.argv
# bag_id = args[1]
# urls = args[2:]
# dir = f'/Users/alecbell/woden/bags/{bag_id}'
# file = f'{dir}.zip'

# # Create zip
# shutil.make_archive(dir, 'zip', dir)

# # Upload zip
# parts = []
# with open(file, 'rb') as f:
#     for i in range(len(urls)):
#         file_data = f.read(1073741824)
#         response = requests.put(urls[i], data=file_data)
#         etag = response.headers['ETag']
#         parts.append({'ETag': etag, 'PartNumber': i+1})

# print(parts)

import json
import urllib.request
import urllib.parse
import shutil
import sys


args = sys.argv
bag_id = args[1]
base_path = args[2]
urls = args[3:]
dir = f'{base_path}/woden/bags/{bag_id}'
file = f'{dir}.zip'

# Create zip
shutil.make_archive(dir, 'zip', dir)

# Upload zip
parts = []
with open(file, 'rb') as f:
    for i in range(len(urls)):
        file_data = f.read(1073741824)
        #response = requests.put(urls[i], data=file_data)
        req = urllib.request.Request(url=urls[i], data=file_data, method='PUT')
        res = urllib.request.urlopen(req)
        etag = res.getheader('ETag').replace('"', '')
        #etag = response.headers['ETag']
        parts.append({'ETag': etag, 'PartNumber': i+1})

print(json.dumps({
    'parts': parts,
    'id': bag_id
}))
