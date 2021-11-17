## nezha_sdc

目前的使用方法

```rust
    unsafe{
        nezha_sdc::mmc_init_test();
    }
    let mut buf = [0u8;512];
    buf.iter_mut().for_each(|ch|*ch = 0x44);
    if let Some(Card) = nezha_sdc::SD_CARD.lock().as_mut(){
        Card.write_blocks(0, 1, &buf);
        Card.read_blocks(0, 1,&buf);
    }
```

