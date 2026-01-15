{
 "model": "robotCP",
 "deviceTypes": [
  {
   "name": "laser",
   "devices": [
    {
     "name": "laser1",
     "isDisplay": true,
     "isEnabled": true,
     "deviceParams": [
      {
       "key": "basic",
       "type": "arrayParam",
       "arrayParam": {
        "params": [
         {
          "key": "x",
          "type": "double",
          "doubleValue": -0.081179290108912561
         },
         {
          "key": "y",
          "type": "double",
          "doubleValue": 0.04306966969789941
         },
         {
          "key": "yaw",
          "type": "double",
          "doubleValue": -1.3958630973524979
         }
        ]
       }
      }
     ]
    },
    {
     "name": "laser2",
     "isDisplay": true,
     "isEnabled": true,
     "deviceParams": [
      {
       "key": "basic",
       "type": "arrayParam",
       "arrayParam": {
        "params": [
         {
          "key": "x",
          "type": "double",
          "doubleValue": -0.034141039363655867
         },
         {
          "key": "y",
          "type": "double",
          "doubleValue": 0.0023926913923868476
         },
         {
          "key": "yaw",
          "type": "double",
          "doubleValue": 1.1276245855809783
         }
        ]
       }
      }
     ]
    },
    {
     "name": "laser",
     "isDisplay": true,
     "isEnabled": true,
     "deviceParams": [
      {
       "key": "basic",
       "type": "arrayParam",
       "arrayParam": {
        "params": [
         {
          "key": "x",
          "type": "double",
          "doubleValue": -0.057847798980125154
         },
         {
          "key": "y",
          "type": "double",
          "doubleValue": 0.019102978988528863
         },
         {
          "key": "yaw",
          "type": "double",
          "doubleValue": -358.76721855957851
         }
        ]
       }
      }
     ]
    }
   ]
  },
  {
   "name": "camera",
   "devices": [
    {
     "name": "mastLidar",
     "isEnabled": true,
     "deviceParams": [
      {
       "key": "basic",
       "type": "arrayParam",
       "arrayParam": {
        "params": [
         {
          "key": "x",
          "type": "double",
          "doubleValue": 0
         },
         {
          "key": "y",
          "type": "double",
          "doubleValue": 0
         },
         {
          "key": "z",
          "type": "double",
          "doubleValue": 0
         },
         {
          "key": "roll",
          "type": "double",
          "doubleValue": 0
         },
         {
          "key": "yaw",
          "type": "double",
          "doubleValue": 0
         },
         {
          "key": "pitch",
          "type": "double",
          "doubleValue": 0
         }
        ]
       }
      }
     ]
    },
    {
     "name": "palletCamera",
     "isDisplay": true,
     "isEnabled": true,
     "deviceParams": [
      {
       "key": "basic",
       "type": "arrayParam",
       "arrayParam": {
        "params": [
         {
          "key": "x",
          "type": "double",
          "doubleValue": 0
         },
         {
          "key": "y",
          "type": "double",
          "doubleValue": -0.0323645773192835
         },
         {
          "key": "z",
          "type": "double",
          "doubleValue": 0.31968044323652212
         },
         {
          "key": "roll",
          "type": "double",
          "doubleValue": -1.9060207133947102
         },
         {
          "key": "yaw",
          "type": "double",
          "doubleValue": 0.22538521036621925
         },
         {
          "key": "pitch",
          "type": "double",
          "doubleValue": 0.48990883643122629
         }
        ]
       }
      }
     ]
    },
    {
     "name": "frontLidar",
     "isDisplay": true,
     "isEnabled": true,
     "deviceParams": [
      {
       "key": "basic",
       "type": "arrayParam",
       "arrayParam": {
        "params": [
         {
          "key": "x",
          "type": "double",
          "doubleValue": 0
         },
         {
          "key": "y",
          "type": "double",
          "doubleValue": 0
         },
         {
          "key": "yaw",
          "type": "double",
          "doubleValue": 0
         },
         {
          "key": "roll",
          "type": "double",
          "doubleValue": -0.702651
         },
         {
          "key": "pitch",
          "type": "double",
          "doubleValue": 71
         },
         {
          "key": "z",
          "type": "double",
          "doubleValue": -0.379678
         }
        ]
       }
      }
     ]
    }
   ]
  },
  {
   "name": "motor",
   "devices": [
    {
     "name": "steer2",
     "isDisplay": true,
     "isEnabled": true,
     "deviceParams": [
      {
       "key": "func",
       "type": "comboParam",
       "comboParam": {
        "childParams": [
         {
          "key": "steer",
          "params": [
           {
            "key": "minAngle",
            "type": "double",
            "doubleValue": -3.51400531883872e-06
           },
           {
            "key": "maxAngle",
            "type": "double",
            "doubleValue": 1.4949510170891855e-06
           }
          ]
         }
        ]
       }
      }
     ]
    }
   ]
  },
  {
   "name": "controller",
   "devices": [
    {
     "name": "controller",
     "isDisplay": true,
     "isEnabled": true,
     "deviceParams": [
      {
       "key": "basic",
       "type": "arrayParam",
       "arrayParam": {
        "params": [
         {
          "key": "x",
          "type": "double",
          "doubleValue": 0
         },
         {
          "key": "y",
          "type": "double",
          "doubleValue": -0.309006
         },
         {
          "key": "qw",
          "type": "double",
          "doubleValue": -0.496513
         },
         {
          "key": "qx",
          "type": "double",
          "doubleValue": 0.507099
         },
         {
          "key": "qy",
          "type": "double",
          "doubleValue": 0.503354
         },
         {
          "key": "qz",
          "type": "double",
          "doubleValue": -0.492909
         },
         {
          "key": "Bax",
          "type": "double",
          "doubleValue": -0.051604
         },
         {
          "key": "Bay",
          "type": "double",
          "doubleValue": -0.010729
         },
         {
          "key": "Baz",
          "type": "double",
          "doubleValue": 0.108596
         }
        ]
       }
      }
     ]
    }
   ]
  },
  {
   "name": "charger",
   "devices": [
    {
     "name": "charger",
     "isDisplay": true,
     "isEnabled": true,
     "deviceParams": [
      {
       "key": "basic",
       "type": "arrayParam",
       "arrayParam": {
        "params": [
         {
          "key": "x",
          "type": "double",
          "doubleValue": 0
         },
         {
          "key": "y",
          "type": "double",
          "doubleValue": 0
         }
        ]
       }
      }
     ]
    }
   ]
  }
 ]
}
