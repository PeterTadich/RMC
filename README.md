# RMC
Resolved Momentum Control (robotics)

## Dependencies

There is 8 dependency 'matrix-computations', 'homogeneous-transformations', 'matlab-javascript', 'elementary-rotations', 'lu-decomposition', 'singular-value-decomposition', 'javascript-data-structures' and 'camera-perspective'.

```bash
https://github.com/PeterTadich/matrix-computations
https://github.com/PeterTadich/homogeneous-transformations
https://github.com/PeterTadich/matlab-javascript
https://github.com/PeterTadich/elementary-rotations
https://github.com/PeterTadich/lu-decomposition
https://github.com/PeterTadich/singular-value-decomposition
https://github.com/PeterTadich/javascript-data-structures
https://github.com/PeterTadich/camera-perspective
```

## Installation

### Node.js

```bash
npm install https://github.com/PeterTadich/RMC
```

### Google Chrome Web browser

No installation required for the Google Chrome Web browser.

## How to use

### Node.js

```js
import * as rmc from 'RMC';
```

### Google Chrome Web browser

```js
import * as rmc from './rmc.mjs';
```

## Examples

### Node.js (server side)

Copy the following code to mini-server.js

```js
//ref: https://www.w3schools.com/nodejs/nodejs_url.asp
var http = require('http');
var url = require('url');
var fs = require('fs');

http.createServer(function (req, res) {
  var q = url.parse(req.url, true);
  var filename = "." + q.pathname;
  fs.readFile(filename, function(err, data) {
    if (err) {
      res.writeHead(404, {'Content-Type': 'text/html'});
      return res.end("404 Not Found");
    } 
    res.writeHead(200, {'Content-Type': 'text/html'});
    res.write(data);
    return res.end();
  });
}).listen(8080); 
```

Copy the following code to quadruped.html

```html
<!DOCTYPE html>
<html lang="en">
    <head>
        <meta charset="UTF-8">
        <title>Quadruped</title>
        <style type="text/css">
            /* IMPORTANT: */
            *{ margin: 0; padding: 0; border: 0;}

            /* IMPORTANT: */
            html, body {
                height:100%;
            }
            
            #container{
                position: relative;
                height: 100%;
                width: 100%;
            }
            #robotCanvas{
                position: absolute;
                height: 100%;
                width: 100%;
            }
        </style>
        <script type="module" src="quadruped.mjs"></script>
    </head>
    <body>
        <div id="container">
            <canvas id="robotCanvas"></canvas>
        </div>
    </body>
</html>
```

Copy the following code to quadruped.mjs

```js
import * as rmc from './rmc.mjs';
import * as cvp from './helpers/cvp.mjs';

cvp.canvasSetup();
rmc.RMC();
```

Then run:

```bash
npm init -y
npm install https://github.com/PeterTadich/camera-perspective https://github.com/PeterTadich/javascript-data-structures https://github.com/PeterTadich/lu-decomposition https://github.com/PeterTadich/singular-value-decomposition https://github.com/PeterTadich/matlab-javascript https://github.com/PeterTadich/elementary-rotations https://github.com/PeterTadich/homogeneous-transformations https://github.com/PeterTadich/matrix-computations
```

Added the following code to the package.json file

```js
  "main": "mini-server.js",
  "scripts": {
    "test": "echo \"Error: no test specified\" && exit 1",
    "start": "node --experimental-modules mini-server.js"
  },
  "type": "module",
```

Full package.json file example

```js
{
  "name": "quadruped",
  "version": "1.0.0",
  "description": "simple quadruped robot",
  "main": "mini-server.js",
  "scripts": {
    "test": "echo \"Error: no test specified\" && exit 1",
    "start": "node --experimental-modules mini-server.js"
  },
  "type": "module",
  "author": "Peter Tadich",
  "license": "MIT",
  "dependencies": {
    "elementary-rotations": "git+https://github.com/PeterTadich/elementary-rotations.git",
    "homogeneous-transformations": "git+https://github.com/PeterTadich/homogeneous-transformations.git",
    "lu-decomposition": "git+https://github.com/PeterTadich/lu-decomposition.git",
    "matlab-javascript": "git+https://github.com/PeterTadich/matlab-javascript.git",
    "matrix-computations": "git+https://github.com/PeterTadich/matrix-computations.git",
    "singular-value-decomposition": "git+https://github.com/PeterTadich/singular-value-decomposition.git",
    "javascript-data-structures": "git+https://github.com/PeterTadich/javascript-data-structures.git",
    "camera-perspective": "git+https://github.com/PeterTadich/camera-perspective.git"
  }
}
```

Now try:

```bash
npm start
```

Start your local server and goto port 8080.

## License

[MIT](LICENSE)