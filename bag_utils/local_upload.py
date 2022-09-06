import argparse
import json
import urllib.request
import urllib.parse
import os
import shutil
from getpass import getpass


def is_bag(path):
    return path.split('/')[-1].isdigit() and 'metadata.yaml' in os.listdir(path)

def refresh_access(refresh):
    data = urllib.parse.urlencode({ "refresh": refresh }).encode()
    req =  urllib.request.Request("https://api.woeden.com/auth/refresh/", data=data)
    resp = urllib.request.urlopen(req)
    resp = json.loads(resp.read())
    return resp['access']

def upload_bag(id, dir, access):
    data = urllib.parse.urlencode({ 'manual': True }).encode()
    req =  urllib.request.Request(f"https://api.woeden.com/bag/{id}/upload/", data=data)
    req.add_header('Authorization', f'Bearer {access}')
    resp = urllib.request.urlopen(req)
    urls = json.loads(resp.read())['urls']

    archive = f'{dir}.zip'
    shutil.make_archive(dir, 'zip', dir)
    parts = []
    with open(archive, 'rb') as f:
        for i in range(len(urls)):
            file_data = f.read(1073741824)
            req = urllib.request.Request(url=urls[i], data=file_data, method='PUT')
            res = urllib.request.urlopen(req)
            etag = res.getheader('ETag').replace('"', '')
            parts.append({'ETag': etag, 'PartNumber': i+1})
    return parts

def mark_uploaded(id, parts, access):
    # parts = urllib.parse.urlencode(parts).encode()
    data = urllib.parse.urlencode({ "parts": json.dumps(parts) }).encode()
    req =  urllib.request.Request(f"https://api.woeden.com/bag/{id}/uploaded/", data=data)
    req.add_header('Authorization', f'Bearer {access}')
    urllib.request.urlopen(req)

parser = argparse.ArgumentParser(description='Upload bags to Woeden')
parser.add_argument('dir', type=str, help="Mount path of external storage device")
parser.add_argument('--email', type=str, help='Account email')
args = parser.parse_args()

email = args.email if args.email is not None else input("Email: ")
password = getpass()

print("Logging in...")

data = urllib.parse.urlencode({ "username": email, "password": password }).encode()
req =  urllib.request.Request("https://api.woeden.com/auth/login/", data=data)
resp = urllib.request.urlopen(req)
resp = json.loads(resp.read())

print("Successfully logged in.\n")

ACCESS = resp['access']
REFRESH = resp['refresh']

print("Searching for bags that have not been uploaded...")

bag_paths = {}
paths_to_search = [args.dir, f'{args.dir}/woeden', f'{args.dir}/woeden/bags']
for path in paths_to_search:
    for dir in os.listdir(path):
        if is_bag(f'{path}/{dir}'):
            bag_paths[int(dir)] = f'{path}/{dir}'
    if len(bag_paths) > 0:
        break

if len(bag_paths.keys()) == 0:
    print("Could not find bags. Please verify the device is mounted at the specified directory.")
    exit()

bags_req =  urllib.request.Request("https://api.woeden.com/bag/")
bags_req.add_header('Authorization', f'Bearer {ACCESS}')
bags_resp = urllib.request.urlopen(bags_req)
bags_to_upload = {}
for bag in json.loads(bags_resp.read()):
    if bag['upload_status'] == 'NOT_UPLOADED' and bag['id'] in bag_paths:
        bags_to_upload[bag['id']] = bag
        bags_to_upload[bag['id']]['path'] = bag_paths[bag['id']]

if len(bags_to_upload.keys()) == 0:
    print("No bags to upload.")
    exit()

print("The following bags have not been uploaded:")
for id in bags_to_upload.keys():
    print("  - " + bags_to_upload[id]['name'])
upload_all = input("Do you wish to upload them all? (y/n): ")

if upload_all not in ('y', 'n'):
    print("Please enter a valid response.")
    exit()

if upload_all == 'n':
    for id in bags_to_upload.keys():
        upload = input(f"Upload {bags_to_upload[id]['name']}? (y/n): ")
        if upload not in ('y', 'n'):
            print("Please enter a valid response.")
            exit()
        if upload == 'n':
            del bags_to_upload[id]

ids = list(bags_to_upload.keys())
for i in range(len(ids)):
    bag = bags_to_upload[ids[i]]
    id = bag['id']
    path = bag['path']
    try:
        parts = upload_bag(id, path, ACCESS)
        mark_uploaded(id, parts, ACCESS)
    except:
        ACCESS = refresh_access(REFRESH)
        parts = upload_bag(id, path, ACCESS)
        mark_uploaded(id, parts, ACCESS)
    print(f"Uploaded {i+1}/{len(ids)}: {bag['name']}")
