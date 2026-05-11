```bash
rostopic echo /rosout --filter "m._connection_header['callerid']=='/fsm_node'" | grep "msg: "
```