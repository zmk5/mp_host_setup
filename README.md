# mp_host_setup
Host computer setup for interfacing with a Mini Pupper over a network

```bash
~$ docker build -t mp:host .
```

```bash
~$ rocker --user --x11 --nvidia mp:host
```