import json
import os
import urllib.request
import urllib.parse
import stream_zip
import sys

from datetime import datetime
from stream_zip import stream_zip, ZIP_64


args = sys.argv
bag_id = args[1]
base_path = args[2]
urls = args[3:]
dir = f'{base_path}/woeden/bags/{bag_id}'

def unzipped_files():
    modified_at = datetime.now()
    perms = 0o600

    def get_bytes(file):
        with open(f'{dir}/{file}', 'rb') as f:
            yield f.read(1073741824)

    for file in os.listdir(dir):
        yield file, modified_at, perms, ZIP_64, get_bytes(file)

parts = []
i = 1
chunks = stream_zip(unzipped_files())
for url, chunk in zip(urls, chunks):
    req = urllib.request.Request(url=url, data=chunk, method='PUT')
    res = urllib.request.urlopen(req)
    etag = res.getheader('ETag').replace('"', '')
    parts.append({'ETag': etag, 'PartNumber': i})
    i += 1

print(json.dumps({
    'parts': parts,
    'id': bag_id
}))
