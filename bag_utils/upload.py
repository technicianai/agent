import json
import os
import urllib.request
import urllib.parse
import stream_zip
import sys

from datetime import datetime
from stream_zip import stream_zip, ZIP_64


args = sys.argv
bag_uuid = args[1]
base_path = args[2]
urls = args[3:]
dir = f'{base_path}/woeden/bags/{bag_uuid}'

def unzipped_files():
    modified_at = datetime.now()
    perms = 0o600

    def get_bytes(file):
        with open(f'{dir}/{file}', 'rb') as f:
            data = f.read(1073741824)
            while data:
                yield data
                data = f.read(1073741824)

    for file in os.listdir(dir):
        yield file, modified_at, perms, ZIP_64, get_bytes(file)

def upload_chunk(gb_chunk, part_no):
    req = urllib.request.Request(url=urls.pop(0), data=gb_chunk, method='PUT')
    res = urllib.request.urlopen(req)
    etag = res.getheader('ETag').replace('"', '')
    parts.append({'ETag': etag, 'PartNumber': part_no})

parts = []
i = 1
chunks = stream_zip(unzipped_files())

# Upload 1 GB at a time
chunk = next(chunks)
gb_chunk = bytearray()
while chunk:
    if len(gb_chunk) + len(chunk) >= 1073741824:
        upload_chunk(gb_chunk, i)
        i += 1
        gb_chunk = bytearray(chunk)
    else:
        gb_chunk.extend(chunk)
    try:
        chunk = next(chunks)
    except StopIteration:
        break

# Upload remaining data
upload_chunk(gb_chunk, i)

print(json.dumps({
    'parts': parts,
    'bag_uuid': bag_uuid
}))
