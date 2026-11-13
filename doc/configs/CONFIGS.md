# CONFIGS.md

hardware configs from control hubs
these should align with README.md and their changes should be tracked with version control

# how to get configs from control hub

```sh
adb shell 'ls /sdcard/FIRST/*.xml' | tr -d '\r' | xargs -I{} adb pull {} .
```

# how to install configs onto control hub
```sh
`adb push <config-name>.xml /sdcard/FIRST/`
```
