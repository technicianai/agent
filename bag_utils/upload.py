import json
import urllib.request
import urllib.parse
import shutil
import sys


args = sys.argv
bag_id = args[1]
base_path = args[2]
urls = args[3:]
dir = f'{base_path}/woeden/bags/{bag_id}'
file = f'{dir}.zip'

shutil.make_archive(dir, 'zip', dir)

parts = []
with open(file, 'rb') as f:
    for i in range(len(urls)):
        file_data = f.read(1073741824)
        req = urllib.request.Request(url=urls[i], data=file_data, method='PUT')
        res = urllib.request.urlopen(req)
        etag = res.getheader('ETag').replace('"', '')
        parts.append({'ETag': etag, 'PartNumber': i+1})

print(json.dumps({
    'parts': parts,
    'id': bag_id
}))
