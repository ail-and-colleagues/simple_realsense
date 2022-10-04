# simple_realsense
## preparement
```
pip install -r requirements.txt
```

## capture_and_record.py
```
usage: capture_and_record.py [-h] -s SAVE_PER_SEC

capture and record image and depth_map

optional arguments:
  -h, --help            show this help message and exit
                        saving rate specified like frame/save_per_sec
```
realsenseから得たデータをキャプチャしつつ、指定したインターバルで記録する。例えば、`capture_and_record.py -s 5`、あるいは`capture_and_record.py --save_per_sec 5`のように指定すると、1秒間に5回記録する。

記録は**log**フォルダの中に作成された**西暦-月-日_時-分**フォルダに**西暦-月-日_時-分-ミリ秒-{img / depth}.png**として記録される。