const char microbit_html[] ="<html>\n"
"<head>\n"
"<meta charset=\"UTF-8\">\n"
"<title>micro:bit information</title>\n"
"<style type='text/css'>\n"
"html{position:relative; min-height:100%;}\n"
"body{font-family:Verdana, sans-serif; font-size:14px;padding:0px;margin:0px;}\n"
"h1{padding-left:5px;width:100%;box-sizing:border-box;color:rgb(0, 0, 0);display:block;font-family:'Segoe UI', Arial, sans-serif;font-size:36px;font-weight:normal;height:54px;line-height:54px;margin-bottom:30px;margin-left:0px;margin-right:0px;margin-top:10px;width:757.156px;word-wrap:break-word;width:100%;border-bottom: 1px solid #ddd;}\n"
"\n"
".center{margin: auto; text-align: center;}\n"
"\n"
".navbar{padding:10px;background-color: #000;}\n"
".navbar h2{color:#fff;}\n"
"\n"
".page-content{max-width: 960px; margin:0 auto;}\n"
"\n"
"#clipboard{max-width: 100%;display: block;width: 100%;height:auto;padding: 6px 12px;font-size: 14px;line-height: 1.42857143;color: #555;background-color: #fff;background-image: none;border: 1px solid #ccc;border-radius: 4px;-webkit-box-shadow: inset 0 1px 1px rgba(0,0,0,.075);box-shadow: inset 0 1px 1px rgba(0,0,0,.075);-webkit-transition: border-color ease-in-out .15s,-webkit-box-shadow ease-in-out .15s;-o-transition: border-color ease-in-out .15s,box-shadow ease-in-out .15s;transition: border-color ease-in-out .15s,box-shadow ease-in-out .15s;}\n"
"\n"
"#fs{width:100%; text-align: center;}\n"
"#fs table{margin-bottom:40px;table-layout: fixed; border-collapse: collapse; width: 100%;vertical-align: middle;text-align: center;}\n"
"#fs table .error{padding-top: 20px;}\n"
"\n"
".popup-wrapper{display: none;position:absolute;top:0;bottom:0;left:0;right:0;overflow:hidden;z-index: 10;background: rgba(0,0,0,0.7);}\n"
".popup{max-height: calc(100% - 100px);position: fixed;top: 50%;left: 50%;transform: translate(-50%, -50%);}\n"
".info-wrapper{width:500px; height:300px; overflow: scroll; background-color: #fff;border-radius: 15px;padding: 20px;}\n"
"\n"
".padded{margin:5px;}\n"
".btn{text-decoration: none;display: inline-block;padding: 6px 12px;margin-bottom: 0;font-size: 14px;font-weight: 400;line-height: 1.42857143;text-align: center;white-space: nowrap;vertical-align: middle;-ms-touch-action: manipulation;touch-action: manipulation;cursor: pointer;-webkit-user-select: none;-moz-user-select: none;-ms-user-select: none;user-select: none;background-image: none;border: 1px solid transparent;border-radius: 4px;}\n"
".btn-default:hover{text-decoration: none;color: #333;background-color: #e6e6e6;border-color: #adadad;}\n"
".btn-default:focus{\n"
"    text-decoration: none;\n"
"}\n"
".btn-default{text-decoration: none;color: #333 !important;background-color: #fff;border-color: #ccc;}\n"
"button, html input[type=button], input[type=reset], input[type=submit] {-webkit-appearance: button;cursor: pointer;}\n"
".btn-primary:hover{color: #fff;background-color: #286090;border-color: #204d74;}\n"
".btn-primary{color: #fff;background-color: #337ab7;border-color: #2e6da4;}\n"
".btn-success{color: #fff;background-color: #5cb85c;border-color: #5cb85c;}\n"
".btn-success:hover{color: #fff;background-color: #449d44;border-color: #419641;}\n"
"\n"
"a:hover, a:focus {color: #00526e;text-decoration: underline;}\n"
"a:active, a:hover {outline: 0;}\n"
"a{color: #008cba;text-decoration: none;}\n"
"a{background: transparent;}\n"
"\n"
".loader {margin: 10px auto;font-size: 10px;position: relative;background-size: 100%; background-color: #fff;background-repeat: no-repeat;-webkit-transform: translateZ(0);-ms-transform: translateZ(0);transform: translateZ(0);-webkit-animation: load8 1.1s infinite linear;animation: load8 1.1s infinite linear;}\n"
".loader,.loader:after {width: 5em;height: 5em;}\n"
"@-webkit-keyframes load8 {0%{-webkit-transform: rotate(0deg);transform: rotate(0deg);}100%{-webkit-transform: rotate(360deg);transform: rotate(360deg);}}\n"
"@keyframes load8 {0%{-webkit-transform: rotate(0deg);transform: rotate(0deg);}100%{-webkit-transform: rotate(360deg);transform: rotate(360deg);}}\n"
"\n"
".ellipsis-loader:after { font-size:48px;overflow: hidden;display: inline-block;vertical-align: bottom;-webkit-animation: ellipsis-inc steps(4,end) 1500ms infinite;animation: ellipsis-inc steps(4,end) 1500ms infinite;content: \"\\2026\";width: 0px; }\n"
"@keyframes ellipsis-inc { to { width: 48px; } }\n"
"@-webkit-keyframes ellipsis-inc { to { width: 48px; } }\n"
"\n"
".speech-bubble {float:left; text-align:left; font-size:18px;position: relative;padding: 15px;margin: 1em 0 3em;border: 5px solid black;color: #333;background: #fff;-webkit-border-radius: 10px;-moz-border-radius: 10px;border-radius: 10px;}\n"
".bubble-align {margin-left: 30px;}\n"
".speech-bubble:before {content: \"\";position: absolute;bottom: -20px;left: 40px;border-width: 20px 20px 0;border-style: solid;border-color: #000 transparent;display: block;width: 0;}\n"
".speech-bubble:after {content: \"\";position: absolute;bottom: -13px;left: 47px;border-width: 13px 13px 0;border-style: solid;border-color: #fff transparent;display: block;width: 0;}\n"
".bubble-align:before {top: 10px;bottom: auto;left: -30px;border-width: 15px 30px 15px 0;border-color: transparent #000;}\n"
".bubble-align:after {top: 16px;bottom: auto;left: -21px;border-width: 9px 21px 9px 0;border-color: transparent #fff;}\n"
"@media (max-width: 753px){\n"
"    .bubble-align:before {top: -20px;bottom: auto;right: auto;left: 40px;border-width: 0 20px 20px;border-color: #000 transparent;}\n"
"    .bubble-align:after {top: -13px;bottom: auto;right: auto;left: 47px;border-width: 0 13px 13px;border:default;border-color: #fff transparent;}\n"
"}\n"
".bubble-align-middle{box-sizing: border-box; width:100% !important;}\n"
".bubble-align-middle:before {top: -20px;bottom: auto;left: 226px;border-width: 0 20px 20px;border:default;border-color:  #000 transparent;}\n"
".bubble-align-middle:after {top: -13px;bottom: auto;left: 233px;border-width: 0 13px 13px;border:default;border-color: #fff transparent;}\n"
"</style>\n"
"<script>\n"
"var files = [];\n"
"var asyncCount = 0;\n"
"\n"
"var MICROBIT_NAME_LENGTH = 5\n"
"var MICROBIT_NAME_CODE_LETTERS = 5\n"
"\n"
"var codebook = [\n"
"    ['z', 'v', 'g', 'p', 't'],\n"
"    ['u', 'o', 'i', 'e', 'a'],\n"
"    ['z', 'v', 'g', 'p', 't'],\n"
"    ['u', 'o', 'i', 'e', 'a'],\n"
"    ['z', 'v', 'g', 'p', 't']\n"
"]\n"
"\n"
"function isIE() {\n"
"    var userAgent = window.navigator.userAgent;\n"
"\n"
"    if (userAgent.indexOf(\"MSIE \") > 0 || !!navigator.userAgent.match(/Trident.*rv\\:11\\./) || /Edge\\/12./i.test(navigator.userAgent))\n"
"        return true;\n"
"\n"
"    return false;\n"
"}\n"
"\n"
"function ascii2string(ascii_ar) {\n"
"    var str = \"\";\n"
"    for(x in ascii_ar) {\n"
"        if(ascii_ar[x] == 0) break;\n"
"        str += String.fromCharCode(ascii_ar[x]);\n"
"    }\n"
"    return str;\n"
"}\n"
"\n"
"function makePageBlob(id, len) {\n"
"    var ar = new Uint8Array(fl.slice(id*1024,(id)*1024+len));\n"
"    return new Blob([ar]);\n"
"}\n"
"\n"
"function makeBlob(blocks,size) {\n"
"    var fileBlob = new Blob();\n"
"    for(x in blocks){\n"
"        if(blocks[x] == 0xFF)\n"
"            return fileBlob;\n"
"\n"
"        fileBlob = new Blob([fileBlob,makePageBlob(blocks[x]+1,Math.min(1024,size))]);\n"
"\n"
"        size-=1024;\n"
"    }\n"
"\n"
"    return fileBlob;\n"
"}\n"
"\n"
"function getFile(id) {\n"
"\n"
"    var ofs = 80*id;\n"
"\n"
"    if(fl[ofs+19] & 0x80)\n"
"        return 0;\n"
"\n"
"    var size = fl[ofs+16]+\n"
"        (fl[ofs+17]<<8)+\n"
"        (fl[ofs+18]<<16)+\n"
"        (fl[ofs+19]<<24);\n"
"\n"
"    var obj = {\n"
"        file:new Blob([makeBlob(fl.slice(ofs+20,ofs+80), size)]),\n"
"        name:(ascii2string(fl.slice((ofs), (ofs)+14))),\n"
"        size:size,\n"
"        data:\"\",\n"
"        dataURL:\"\"\n"
"    }\n"
"\n"
"    return obj;\n"
"}\n"
"\n"
"function autoCopy(){\n"
"    var clipboard = document.querySelector('#clipboard');\n"
"    clipboard.select();\n"
"\n"
"    var success = false;\n"
"\n"
"    try {\n"
"        success = document.execCommand('copy');\n"
"    } catch(err) {\n"
"    }\n"
"    window.getSelection().removeAllRanges();\n"
"\n"
"    return success;\n"
"}\n"
"\n"
"function copyToClipboard(idx, button){\n"
"    var clipboard = document.getElementById(\"clipboard\")\n"
"\n"
"\n"
"    clipboard.value = files[idx].data;\n"
"\n"
"    document.getElementById(\"popup\").style.display=\"block\";\n"
"\n"
"    if(autoCopy()){\n"
"        document.getElementById(\"popup\").style.display=\"none\";\n"
"        button.className += \" btn-success\"\n"
"        button.disabled = true;\n"
"        button.innerHTML = \"Copied!\"\n"
"\n"
"        setTimeout(function(){\n"
"            button.className = button.className.replace(' btn-success','')\n"
"            button.innerHTML = \"Copy to clipboard\"\n"
"            button.disabled = false;\n"
"        },2000)\n"
"    }\n"
"}\n"
"\n"
"function transposeName(serialNumber) {\n"
"\n"
"    var name = \"\";\n"
"\n"
"\t// Derive our name from the nrf51822's unique ID.\n"
"    var n = Math.abs(serialNumber);\n"
"\n"
"    var ld = 1;\n"
"    var d = MICROBIT_NAME_CODE_LETTERS;\n"
"    var h;\n"
"\n"
"    for (var i = 0; i < MICROBIT_NAME_LENGTH; i++)\n"
"    {\n"
"        h = Math.round((n % d) / ld);\n"
"        n -= h;\n"
"        d *= MICROBIT_NAME_CODE_LETTERS;\n"
"        ld *= MICROBIT_NAME_CODE_LETTERS;\n"
"        console.log(i,h,codebook, codebook[i], codebook[i][h])\n"
"        name = codebook[i][h] + name;\n"
"    }\n"
"\n"
"    return name;\n"
"}\n"
"\n"
"function buildDetails() {\n"
"    var html = 'Hello, my name is: <b>' + transposeName(parseInt(details.serialNumber)) + \"</b>!<br/><br/>\"\n"
"    html += \"I'm running DAPLink firmware version: <b>\" + details.version + \"</b><br/><br/>\"\n"
"    html += \"BLE Address: \" +\n"
"        details.BLE_address.slice(2,4) + \":\" +\n"
"        details.BLE_address.slice(4,6) + \":\" +\n"
"        details.BLE_address.slice(6,8) + \":\" +\n"
"        details.BLE_address.slice(8,10) + \":\" +\n"
"        details.BLE_address.slice(10,12) + \":\" +\n"
"        details.BLE_address.slice(12,14) + \"</b><br/><br/>\";\n"
"    html += 'Read more about the <a href=\"https://lancaster-university.github.io/microbit-docs\">runtime</a>.'\n"
"    document.getElementById(\"mbit-speech\").innerHTML = html;\n"
"}\n"
"\n"
"function loadDetails(){\n"
"    var sc = document.createElement('script');\n"
"    sc.src='SPEC.TXT'\n"
"    document.getElementsByTagName(\"head\")[0].appendChild(sc);\n"
"\n"
"    sc.onload = sc.onreadystatechange = function(){\n"
"        buildDetails();\n"
"    }\n"
"}\n"
"\n"
"function loadFlash() {\n"
"\n"
"    var sc = document.createElement('script');\n"
"    sc.src='FLASHJS.TXT'\n"
"    document.getElementsByTagName(\"head\")[0].appendChild(sc);\n"
"    sc.onload = sc.onreadystatechange=function(){\n"
"        buildIF();\n"
"    };\n"
"}\n"
"\n"
"function toObjectURL(file){\n"
"    var url = window.URL || window.webkitURL;\n"
"    return url.createObjectURL(file);\n"
"}\n"
"\n"
"//will return the number of files in the file system eventually...\n"
"function numberOfFiles(){\n"
"    return 10;\n"
"}\n"
"\n"
"function readFile(index){\n"
"    var clipboardTransposer = new FileReader();\n"
"    var downloadTransposer = new FileReader();\n"
"\n"
"    clipboardTransposer.addEventListener(\"loadend\", function(e, extra){\n"
"        if(typeof e.srcElement == \"undefined\")\n"
"            files[index].data = clipboardTransposer.result;\n"
"        else\n"
"            files[index].data = e.srcElement.result\n"
"        asyncCount--;\n"
"    });\n"
"\n"
"    downloadTransposer.addEventListener(\"loadend\", function(e){\n"
"        if(typeof e.srcElement == \"undefined\")\n"
"            files[index].dataURL = downloadTransposer.result;\n"
"        else\n"
"            files[index].dataURL = e.srcElement.result\n"
"\n"
"        files[index].dataURL = files[index].dataURL.replace('data:;', 'data:attachment/csv;')\n"
"\n"
"        console.log(files[index].dataURL);\n"
"\n"
"        asyncCount--;\n"
"    });\n"
"\n"
"    asyncCount += 2;\n"
"\n"
"    if(files[index].name.indexOf(\".bin\") > 0)\n"
"        clipboardTransposer.readAsBinaryString(files[index].file);\n"
"    else\n"
"        clipboardTransposer.readAsText(files[index].file);\n"
"\n"
"    downloadTransposer.readAsDataURL(files[index].file);\n"
"}\n"
"\n"
"function getFiles(){\n"
"    var i = 0;\n"
"\n"
"    for(i =0; i < numberOfFiles(); i++)\n"
"    {\n"
"        var file = getFile(i);\n"
"        //i++;\n"
"\n"
"        if(file == 0) continue;\n"
"        files.push(file);\n"
"\n"
"        readFile(files.length - 1);\n"
"    }\n"
"\n"
"\n"
"    /*while(files.length < numberOfFiles()){\n"
"        var file = getFile(i);\n"
"        i++;\n"
"\n"
"        if(file == 0) continue;\n"
"        files.push(file);\n"
"\n"
"        readFile(files.length - 1);\n"
"    }*/\n"
"}\n"
"\n"
"function buildInterface(){\n"
"    var str = \"<table>\";\n"
"    str += '<col width=\"30%\"/><col width=\"25%\"/><col width=\"22.5%\"/><col width=\"22.5%\"/>'\n"
"    str += \"<tr><th>Name</th><th>Size</th></tr>\"\n"
"\n"
"    for(var i = 0; i < files.length; i++) {\n"
"        var file = files[i];\n"
"        str += \"<tr><td>\"+file.name+\"</td><td>\"+file.size+\"KB</td><td>\"+ ((file.data.length > 0) ?\"<button type='button' class='btn btn-primary padded' onClick='copyToClipboard(\"+i+\", this)'>Copy to Clipboard</button>\":\"\") + \"</td><td>\"+ ((file.dataURL.length > 0)? \"<a type='button' class='btn btn-default padded' download='\" + file.name + \"' href='\"+file.dataURL+\"' onclick='return ieDownload(\" + i + \", this);'>Download</a>\" : \"\" ) + \"</td></tr>\";\n"
"    }\n"
"\n"
"    if(files.length == 0)\n"
"        str+=\"<tr><td class='error' colspan='100%'>You haven't made any files!</td>\"\n"
"\n"
"    document.getElementById(\"fs\").innerHTML = str+\"</table>\";\n"
"}\n"
"\n"
"function loaded(){\n"
"    if(asyncCount <= 0){\n"
"        buildInterface();\n"
"        return;\n"
"    }\n"
"    setTimeout(loaded,500);\n"
"}\n"
"\n"
"function buildIF(){\n"
"    getFiles();\n"
"    loaded();\n"
"}\n"
"\n"
"function backgroundClick(el, e){\n"
"    var event;\n"
"\n"
"    if(typeof e == \"undefined\")\n"
"        event = window.event\n"
"    else\n"
"        event = e\n"
"\n"
"    if(event.target == el)\n"
"        el.style.display = \"none\"\n"
"}\n"
"\n"
"function ieDownload(idx, e){\n"
"    var event;\n"
"\n"
"    if(typeof e == \"undefined\")\n"
"        event = window.event\n"
"    else\n"
"        event = e\n"
"\n"
"    if(isIE())\n"
"    {\n"
"        if(event.preventDefault)\n"
"            event.preventDefault()\n"
"        else\n"
"            event.returnValue = false;\n"
"\n"
"        console.log(\"IE, saving as blob\");\n"
"        var ret = navigator.msSaveBlob(files[idx].file, files[idx].name);\n"
"        console.log(\"RET: \",ret);\n"
"        return false;\n"
"    }\n"
"\n"
"    console.log(\"Not IE, hooray!\");\n"
"    return true;\n"
"}\n"
"</script>\n"
"</head>\n"
"<body onLoad='loadFlash(),loadDetails()'>\n"
"    <div class=\"navbar\">\n"
"        <div class=\"center\">\n"
"            <svg width=\"168\" height=\"54\" version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" x=\"0px\" y=\"0px\"\n"
"            \t viewBox=\"0 0 164 36\" style=\"enable-background:new 0 0 164 36;\" xml:space=\"preserve\">\n"
"                <style type=\"text/css\">\n"
"                \t.st0{fill:#FFFFFF;}\n"
"                \t.st1{font-family:'GTWalsheim';}\n"
"                \t.st2{font-size:21.4675px;}\n"
"                </style>\n"
"                <g id=\"Logo\">\n"
"                \t<g>\n"
"                \t\t<g>\n"
"                \t\t\t<path class=\"st0\" d=\"M70.4,23.6h-3.2v-9.4c0-2.1-1.1-3.6-2.7-3.6c-1.6,0-2.8,1.5-2.8,3.6v9.4h-3.3v-9.4c0-2.4-1.4-3.6-2.8-3.6\n"
"                \t\t\t\tc-1.6,0-2.8,1.5-2.8,3.6v9.4h-3.2v-9.2c0-4.2,2.5-7.1,6-7.1c1.7,0,3.1,0.7,4.3,2.2c1.2-1.4,2.7-2.2,4.4-2.2\n"
"                \t\t\t\tc3.5,0,5.9,2.9,5.9,7.1V23.6z\"/>\n"
"                \t\t</g>\n"
"                \t\t<g>\n"
"                \t\t\t<path class=\"st0\" d=\"M76.3,23.6h-3.3v-16h3.3V23.6z M74.7,5.3c-1.2,0-2.1-1-2.1-2.2c0-1.2,0.9-2.2,2.1-2.2c1.2,0,2.1,1,2.1,2.2\n"
"                \t\t\t\tC76.8,4.3,75.8,5.3,74.7,5.3z\"/>\n"
"                \t\t</g>\n"
"                \t\t<g>\n"
"                \t\t\t<path class=\"st0\" d=\"M86.6,24c-2.1,0-4.1-0.9-5.6-2.5c-1.5-1.5-2.3-3.6-2.3-5.9c0-2.3,0.8-4.4,2.3-5.9c1.5-1.6,3.5-2.5,5.6-2.5\n"
"                \t\t\t\tc2.3,0,4.2,0.8,5.7,2.4l0.4,0.4l-2.3,2.5L90,12.1c-1.1-1-2.1-1.5-3.3-1.5c-2.6,0-4.7,2.2-4.7,5c0,2.8,2.1,5,4.7,5\n"
"                \t\t\t\tc1.2,0,2.3-0.5,3.3-1.5l0.4-0.4l2.4,2.3l-0.5,0.5C90.6,23.1,88.7,24,86.6,24z\"/>\n"
"                \t\t</g>\n"
"                \t\t<g>\n"
"                \t\t\t<path class=\"st0\" d=\"M98.4,23.6H95v-8c0-5.1,2.1-7.6,6.9-8l0.7-0.1v3.3L102,11c-2.6,0.3-3.6,1.5-3.6,4.4V23.6z\"/>\n"
"                \t\t</g>\n"
"                \t\t<g>\n"
"                \t\t\t<path class=\"st0\" d=\"M111.5,24c-2.1,0-4.1-0.9-5.6-2.5c-1.5-1.6-2.3-3.7-2.3-5.9c0-2.2,0.8-4.3,2.3-5.9c1.5-1.6,3.5-2.5,5.6-2.5\n"
"                \t\t\t\tc2.1,0,4.1,0.9,5.6,2.5c1.5,1.5,2.3,3.6,2.3,5.9c0,2.3-0.8,4.4-2.3,5.9C115.6,23.1,113.6,24,111.5,24z M111.5,10.6\n"
"                \t\t\t\tc-2.5,0-4.5,2.3-4.5,5c0,2.8,2,5,4.5,5c2.6,0,4.6-2.2,4.6-5C116,12.8,114,10.6,111.5,10.6z\"/>\n"
"                \t\t</g>\n"
"                \t\t<g>\n"
"                \t\t\t<path class=\"st0\" d=\"M124.7,23.6c-1.2,0-2.2-1-2.2-2.3c0-1.3,1-2.4,2.2-2.4c1.2,0,2.3,1.1,2.3,2.4\n"
"                \t\t\t\tC127,22.5,125.9,23.6,124.7,23.6z\"/>\n"
"                \t\t</g>\n"
"                \t\t<g>\n"
"                \t\t\t<path class=\"st0\" d=\"M124.7,11.9c-1.2,0-2.2-1-2.2-2.3c0-1.3,1-2.4,2.2-2.4c1.2,0,2.3,1.1,2.3,2.4\n"
"                \t\t\t\tC127,10.8,125.9,11.9,124.7,11.9z\"/>\n"
"                \t\t</g>\n"
"                \t\t<g>\n"
"                \t\t\t<path class=\"st0\" d=\"M137.9,24c-4.7,0-7.9-3.8-7.9-9.2c0-0.3,0-0.7,0-1.1L130,0h3.2v8.8c1.4-1,2.9-1.5,4.6-1.5\n"
"                \t\t\t\tc2.1,0,4.1,0.9,5.6,2.4c1.5,1.6,2.3,3.7,2.3,5.9c0,2.3-0.8,4.4-2.3,5.9C142,23.1,140,24,137.9,24z M137.9,10.6\n"
"                \t\t\t\tc-2.6,0-4.7,2.3-4.7,5c0,2.8,2.1,5,4.7,5c2.6,0,4.7-2.2,4.7-5C142.6,12.8,140.5,10.6,137.9,10.6z\"/>\n"
"                \t\t</g>\n"
"                \t\t<g>\n"
"                \t\t\t<path class=\"st0\" d=\"M151.8,23.6h-3.3v-16h3.3V23.6z M150.2,5.3c-1.2,0-2.1-1-2.1-2.2c0-1.2,0.9-2.2,2.1-2.2c1.2,0,2.1,1,2.1,2.2\n"
"                \t\t\t\tC152.3,4.3,151.3,5.3,150.2,5.3z\"/>\n"
"                \t\t</g>\n"
"                \t\t<g>\n"
"                \t\t\t<path class=\"st0\" d=\"M164,24l-0.7-0.1c-4.8-1-6.6-3.4-6.6-8.8v-4h-1.4V7.8h1.4V4.3h3.3v3.5h4v3.2h-4v4.5c0,3,1.1,4.5,3.5,4.8\n"
"                \t\t\t\tl0.5,0.1V24z\"/>\n"
"                \t\t</g>\n"
"                \t</g>\n"
"                \t<g>\n"
"                \t\t<path class=\"st0\" d=\"M31.8,18.3c1.4,0,2.6-1.2,2.6-2.7S33.3,13,31.8,13c-1.4,0-2.6,1.2-2.6,2.7S30.4,18.3,31.8,18.3\"/>\n"
"                \t\t<path class=\"st0\" d=\"M12.8,13c-1.4,0-2.6,1.2-2.6,2.7s1.2,2.7,2.6,2.7c1.4,0,2.6-1.2,2.6-2.7S14.3,13,12.8,13\"/>\n"
"                \t\t<path class=\"st0\" d=\"M12.8,7.6c-4.2,0-7.7,3.6-7.7,8c0,4.4,3.5,8,7.7,8h19.2c4.2,0,7.7-3.6,7.7-8c0-4.4-3.5-8-7.7-8H12.8 M32.1,29\n"
"                \t\t\tH12.8C5.8,29,0,23,0,15.7S5.8,2.3,12.8,2.3h19.2c7.1,0,12.8,6,12.8,13.4S39.1,29,32.1,29\"/>\n"
"                \t</g>\n"
"                </g>\n"
"            </svg>\n"
"        </div>\n"
"    </div>\n"
"    <div class=\"page-content\">\n"
"        <h1>About me</h1>\n"
"        <center>\n"
"            <div style=\"display:inline-block;\" class=\"padded\">\n"
"                <svg style=\"float:left;\" version=\"1.0\" xmlns=\"http://www.w3.org/2000/svg\" width=\"197.000000pt\" height=\"152.000000pt\" viewBox=\"0 0 197.000000 152.000000\" preserveAspectRatio=\"xMidYMid meet\">\n"
"                    <g transform=\"translate(0.000000,152.000000) scale(0.100000,-0.100000)\"\n"
"                    fill=\"#000000\" stroke=\"none\">\n"
"                        <path d=\"M400 1486 c-336 -95 -495 -465 -338 -786 40 -83 142 -185 227 -228\n"
"                        122 -61 150 -63 726 -60 570 4 544 1 673 67 69 35 166 133 206 210 167 318 5\n"
"                        709 -329 796 -50 13 -144 15 -590 14 -395 -1 -541 -4 -575 -13z m1148 -244\n"
"                        c66 -35 119 -88 149 -150 23 -47 28 -70 28 -132 -1 -129 -67 -233 -185 -289\n"
"                        l-55 -26 -505 0 -505 0 -57 28 c-249 122 -237 476 19 577 45 18 78 19 558 17\n"
"                        l510 -2 43 -23z\"/>\n"
"                        <path d=\"M484 1051 c-35 -22 -60 -78 -50 -116 10 -40 54 -84 92 -91 73 -14\n"
"                        134 38 134 113 0 38 -5 50 -34 79 -40 40 -92 46 -142 15z\"/>\n"
"                        <path d=\"M1334 1036 c-28 -28 -34 -41 -34 -78 0 -50 19 -82 62 -104 90 -46\n"
"                        197 53 158 146 -18 43 -60 70 -110 70 -34 0 -48 -6 -76 -34z\"/>\n"
"                        <path d=\"M206 202 c-12 -19 18 -87 56 -128 95 -103 270 -93 350 21 16 22 31\n"
"                        57 35 78 l6 37 -47 0 c-43 0 -47 -2 -60 -35 -31 -73 -135 -99 -196 -49 -17 15\n"
"                        -37 39 -46 55 -12 24 -21 29 -54 29 -21 0 -41 -4 -44 -8z\"/>\n"
"                    </g>\n"
"                </svg>\n"
"                <p id=\"mbit-speech\" style=\"float:left; text-align:left; font-size:18px;min-width:100px;\"  class=\"speech-bubble bubble-align \">\n"
"                    <i class=\"ellipsis-loader\"></i>\n"
"                </p>\n"
"            </div>\n"
"        </center>\n"
"        <h1>My Files</h1>\n"
"        <div id='fs'>\n"
"            <div>\n"
"                <div style=\"overflow:hidden;\">\n"
"                    <div class=\"loader\">\n"
"                        <svg version=\"1.0\" xmlns=\"http://www.w3.org/2000/svg\"\n"
"                         width=\"100%\" height=\"100%\" viewBox=\"0 0 197.000000 156.000000\"\n"
"                         preserveAspectRatio=\"xMidYMid meet\">\n"
"                            <g transform=\"translate(0.000000,156.000000) scale(0.100000,-0.100000)\"\n"
"                            fill=\"#000000\" stroke=\"none\">\n"
"                                <path d=\"M395 1521 c-250 -72 -410 -303 -392 -565 17 -241 181 -437 409 -491\n"
"                                93 -22 1044 -22 1137 0 193 46 350 204 396 397 70 296 -103 588 -391 663 -49\n"
"                                12 -145 15 -579 14 -461 0 -527 -2 -580 -18z m1111 -226 c78 -23 143 -80 184\n"
"                                -159 30 -59 34 -77 34 -140 0 -127 -65 -229 -184 -288 -45 -23 -45 -23 -560\n"
"                                -23 l-515 0 -50 27 c-115 60 -177 160 -177 281 0 141 80 254 212 300 65 23\n"
"                                981 25 1056 2z M488 1092 c-62 -38 -73 -125 -21 -176 74 -75 193 -25 193 81 0\n"
"                                38 -5 50 -34 79 -27 27 -42 34 -72 34 -22 0 -51 -8 -66 -18z M1334 1076 c-29\n"
"                                -29 -34 -41 -34 -79 0 -106 119 -156 193 -81 73 73 19 194 -88 194 -29 0 -45\n"
"                                -8 -71 -34z M910 203 c-60 -55 -55 -137 10 -181 65 -44 147 -11 171 68 31 104\n"
"                                -101 185 -181 113z\"/>\n"
"                            </g>\n"
"                        </svg>\n"
"                    </div>\n"
"                </div>\n"
"            </div>\n"
"        </div>\n"
"    </div>\n"
"    <div class=\"popup-wrapper\" id=\"popup\" onClick=\"backgroundClick(this, event)\">\n"
"        <div class=\"popup\">\n"
"            <div class=\"info-wrapper\">\n"
"                <div class=\"center\">\n"
"                    <svg version=\"1.0\" xmlns=\"http://www.w3.org/2000/svg\"\n"
"                     width=\"98.500000pt\" height=\"76.000000pt\" viewBox=\"0 0 197.000000 152.000000\"\n"
"                     preserveAspectRatio=\"xMidYMid meet\">\n"
"                        <g transform=\"translate(0.000000,152.000000) scale(0.100000,-0.100000)\"\n"
"                        fill=\"#000000\" stroke=\"none\">\n"
"                            <path d=\"M423 1505 c-204 -44 -365 -207 -408 -412 -62 -294 103 -572 386 -650\n"
"                            57 -16 118 -18 584 -18 509 0 522 0 590 22 149 47 268 147 330 278 40 85 55\n"
"                            152 55 248 0 154 -53 283 -159 387 -78 77 -142 113 -247 140 -72 19 -110 20\n"
"                            -573 19 -378 0 -511 -4 -558 -14z m1117 -246 c70 -33 126 -89 159 -159 34 -72\n"
"                            36 -181 4 -251 -28 -63 -86 -125 -147 -157 l-51 -27 -525 0 -525 0 -53 29\n"
"                            c-136 76 -197 234 -148 383 31 94 131 183 227 202 24 5 260 9 524 8 l480 -2\n"
"                            55 -26z M470 1091 c-16 -31 -12 -45 25 -81 l34 -33 -34 -39 c-39 -43 -44 -67\n"
"                            -17 -91 27 -24 47 -21 85 13 l34 29 28 -29 c30 -31 63 -38 83 -18 21 21 13 61\n"
"                            -20 95 l-32 33 31 33 c39 42 45 60 27 86 -20 28 -56 26 -94 -5 l-31 -27 -28\n"
"                            27 c-32 31 -76 34 -91 7z M1256 1088 c-22 -30 -21 -32 20 -78 l36 -40 -36 -40\n"
"                            c-41 -46 -42 -48 -20 -78 21 -30 52 -28 89 8 l31 30 29 -30 c33 -35 64 -38 90\n"
"                            -10 23 25 15 57 -24 94 l-30 29 34 39 c39 42 43 59 19 82 -23 23 -54 19 -85\n"
"                            -10 l-28 -27 -31 27 c-39 32 -74 33 -94 4z M913 159 c-101 -89 -109 -103 -73\n"
"                            -139 29 -29 45 -25 95 21 l45 42 45 -42 c24 -22 53 -41 63 -41 23 0 52 30 52\n"
"                            54 0 31 -131 156 -164 156 -3 0 -32 -23 -63 -51z\"/>\n"
"                        </g>\n"
"                    </svg>\n"
"                </div>\n"
"                <div class=\"center\">\n"
"                    <p style=\"box-sizing: border-box; width:100%;\"  class=\"speech-bubble bubble-align-middle\">\n"
"                        Oops, I didn't manage to copy that file to your clipboard.\n"
"                        <br/>\n"
"                        <br/>\n"
"                        You will have copy it yourself:\n"
"                        <br/>\n"
"                        <br/>\n"
"                        <textarea rows=\"10\" id=\"clipboard\">\n"
"                        </textarea>\n"
"                    </p>\n"
"                </div>\n"
"            </div>\n"
"        </div>\n"
"    </div>\n"
"</body>\n"
"</html>";
