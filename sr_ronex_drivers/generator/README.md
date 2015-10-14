# Automatic RoNeX driver generator

This generates a skeleton for the RoNeX drivers. To run it, run the following command, making sure you replace the module name and product id:

```
roscd sr_ronex_drivers/generator
./generate_driver.py -m module_name -p 0x020000009
```
