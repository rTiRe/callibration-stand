const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">

    <style>
        body {
            width: 100%;
            padding: 0;
            margin: 0;
            display: flex;
            align-items: center;
            flex-direction: column;
        }
        header {
            height: 55px;
            width: 100%;
            background-color: #555;
            position: relative;
            color: #eee;
            padding: 5px 15px;
            display: flex;
            flex-direction: row;
            justify-content: center;
            align-items: center;
            box-sizing: border-box;
        }
        .Wrapper {
            display: flex;
            flex-direction: row;
            justify-content: space-between;
            align-items: center;
            max-width: 1280px;
            width: 100%;
        }
        main {
            position: relative;
            padding: 10px 0;
            max-width: 1280px; 
            width: 100%;
            display: flex;
            grid-gap: 10px;
            box-sizing: border-box;
        }
        @media screen and (max-width: 1280px) {
            main {
                padding: 10px;
            }
            .Wrapper {
                padding: 0 10px;
            }
        }
        #settings {
            position: relative;
            width: 70%;
            background-color: rgb(196, 196, 196);
            padding: 10px;
            border-radius: 8px;
            display: inline-flex;
            flex-direction: column;
            grid-gap: 20px;
            box-sizing: border-box;
        }
        #values {
            position: relative;
            width: 30%;
            /*background-color: rgb(196, 196, 196);
            padding: 10px;
            border-radius: 8px;*/
            display: inline-flex;
            flex-direction: column;
            grid-gap: 10px;
            box-sizing: border-box;
        }
        #speedRange {
            position: relative;
            width: 100%;
            display: inline-flex;
            flex-direction: column;
        }
        #speedRange > input {
            position: relative;
            width: 100%;
            max-width: 450px;
        }
        datalist {
            position: relative;
            display: flex;
            justify-content: space-between;
            width: calc(100%+6px);
            max-width: 453px;
            min-width: calc(100%-3px);
        }
        #rotationVector, #modes {
            position: relative;
            width: 100%;
        }
        #rotationVector .btns, #modes .btns {
            display: flex;
            grid-gap: 15px;
        }
        #modes .btns {
            flex-direction: column;
            align-items: flex-start;
            grid-gap: 5px;
        }
        #Power, #rotationVector input, #modes input {
            min-width: 120px;
            height: 35px;
            padding: 5px 22px 5px 22px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-weight: bold;
            color: white;
            box-sizing: border-box;
        }
        #Power:focus, #rotationVector input:focus, #modes input:focus {
            border: 1px solid black;
        }
        #motorSpeed, #motorRot, #tenzo {
            font-weight: bold;
        }

        .settingsSection {
            border-radius: 6px;
            padding: 10px;
            box-sizing: border-box;
            background-color: rgb(204, 204, 204);
        }
        #values > section {
            display: flex;
            flex-direction: column;
            border-radius: 8px;
            padding: 10px;
            box-sizing: border-box;
            background-color: rgb(196, 196, 196);
        }

        @media screen and (max-width: 768px) {
            main {
                flex-direction: column-reverse;
                grid-gap: 8px;
            }
            #settings {
                width: 100%;
                grid-gap: 10px;
            }
            #values {
                width: 100%;
                grid-gap: 8px;
            }
        }
    </style>
    <title>Callibration Stand</title>
</head>
<body>
    <header>
        <div class="Wrapper">
            <h3>Stand HMI</h3>
            <input type="button" id="Power" onclick="powerChange(true)">
        </div>
    </header>
    <main>
        <div id="settings">
            <div id="speedRange" class="settingsSection">
                <label for="range" style="font-size: 23px; font-weight: bold; text-align: left;">Скорость вращения (0-255):</label>
                <input type="range" min="0" max="255" value="0" step="15" list="volsettings" id="range" style="margin-top: 10px;"/>
                <!--<datalist id="volsettings">
                        <option value="0"></option>
                        <option value="100" label="номинальная"></option>
                        <option value="166"></option>
                    </datalist> -->
                <!-- написать про максимальную скорость -->
            </div>
            <div id="rotationVector" class="settingsSection">
                <span style="font-size: 23px; font-weight: bold;">Направление вращения:</span><br>
                <div class="btns" style="margin-top: 10px;">
                    <input type="button" id="rotationVectorBtn" onclick="rotationVectorChange(true)">
                </div>
            </div>
            <div id="modes" class="settingsSection">
                <span style="font-size: 23px; font-weight: bold;">Предустановленные сценарии:</span><br>
                <div class="btns" style="margin-top: 10px;">
                    <input type="button" value="Откл. сценарий" id="m0" style="background-color: lightcoral;" onclick="changeMode(0)">
                    <input type="button" value="Сценарий 1" id="m1" style="background-color: lightseagreen;" onclick="changeMode(1)">
                    <input type="button" value="Сценарий 2" id="m2" style="background-color: steelblue;" onclick="changeMode(2)">
                    <input type="button" value="Сценарий 3" id="m3" style="background-color: lightseagreen;" onclick="changeMode(3)">
                </div>
            </div>
        </div>
        <div id="values">
            <section id="motorSpeed">
                <span style="font-size: 23px; font-weight: bold;">Скорость:</span> <span style="margin-top:10px;"><span id="speedValue" style="font-size: 20px;">???</span> об/сек</span>
            </section>
            <section id="motorRot">
                <span style="font-size: 23px; font-weight: bold;">Направление вращения:</span> <span style="margin-top:10px;"><span id="rotationVectorValue" style="font-size: 20px;">???</span></span>
            </section>
            <section id="tenzo">
              <span style="font-size: 23px; font-weight: bold;">Момент:</span><span style="margin-top:10px;"><span id="gramms" style="font-size: 20px">???</span> Н·см</span>
            </section>
        </div>
    </main>

    <script>
        var range = document.getElementById('range');

        /* Эти переменные запрашивать от сервера!!! */
        var powerV = 0;
        var rangeV = 0;
        var rotationVectorV = 1;
        var activeMode = 0;

        var speedInterval;
        var grammsInterval;

        setInterval(function fload() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                    var rec = this.responseText.split(" ");
                    powerV = rec[0];
                    rangeV = rec[1];
                    rotationVectorV = rec[2];
                    activeMode = rec[3];
                    powerChange(false);
                    rotationVectorChange(false);
                    document.getElementById("range").value = rangeV;
                    console.log(rangeV);
                }
            };
            xhttp.open("GET", "getdata", true);
            xhttp.send();
        }, 500);
        
        function powerChange(k) {
            if(k == true) {
                if(powerV == 0) {
                    document.getElementById("Power").setAttribute("value", "Выключить");
                    document.getElementById("Power").style.backgroundColor = "lightcoral";
                    power(1);
                    if(!speedInterval) speedInterval = setInterval(getspeed, 500);
                    if(!grammsInterval) grammsInterval = setInterval(getgramms, 500);
                }
                else {
                    document.getElementById("Power").setAttribute("value", "Включить");
                    document.getElementById("Power").style.backgroundColor = "limegreen";
                    power(0);
                    if(speedInterval) speedInterval = clearInterval(speedInterval);
                    if(grammsInterval) grammsInterval = clearInterval(grammsInterval);
                    document.querySelector("#speedValue").innerHTML = "0";
                    document.querySelector("#gramms").innerHTML = "0";

                }
            }
            else {
                if(powerV == 0) {
                    document.getElementById("Power").setAttribute("value", "Включить");
                    document.getElementById("Power").style.backgroundColor = "limegreen";
                    if(speedInterval) speedInterval = clearInterval(speedInterval);
                    if(grammsInterval) grammsInterval = clearInterval(grammsInterval);
                    document.querySelector("#speedValue").innerHTML = "0";
                    document.querySelector("#gramms").innerHTML = "0";
                }
                else {
                    document.getElementById("Power").setAttribute("value", "Выключить");
                    document.getElementById("Power").style.backgroundColor = "lightcoral";
                    if(!speedInterval) speedInterval = setInterval(getspeed, 500);
                    if(!grammsInterval) grammsInterval = setInterval(getgramms, 500);
                }
            }
        }

        function rotationVectorChange(k) {
            if(k == true) {
                if(rotationVectorV == 0) {
                    document.getElementById("rotationVectorBtn").setAttribute("value", "перевести в Реверс");
                    document.getElementById("rotationVectorBtn").style.backgroundColor = "#008EB0";
                    document.getElementById("rotationVectorValue").innerHTML = "Аверс";
                    rotationVector(1);
                }
                else {
                    document.getElementById("rotationVectorBtn").setAttribute("value", "перевести в Аверс");
                    document.getElementById("rotationVectorBtn").style.backgroundColor = "#F7941E";
                    document.getElementById("rotationVectorValue").innerHTML = "Реверс";
                    rotationVector(0);
                }
            }
            else {
                if(rotationVectorV == 0) {
                    document.getElementById("rotationVectorBtn").setAttribute("value", "перевести в Аверс");
                    document.getElementById("rotationVectorBtn").style.backgroundColor = "#F7941E";
                    document.getElementById("rotationVectorValue").innerHTML = "Реверс";
                }
                else {
                    document.getElementById("rotationVectorBtn").setAttribute("value", "перевести в Реверс");
                    document.getElementById("rotationVectorBtn").style.backgroundColor = "#008EB0";
                    document.getElementById("rotationVectorValue").innerHTML = "Аверс";
                }
            }
        }

        function power(k) {
            if(k != powerV) {
                powerV = k;
                sendRequest(0, k);
            }
        }
        range.addEventListener('change', function () {
            /* после отжатия кнопки мыши */
            if(this.value != rangeV) {
                rangeV = this.value;
                sendRequest(1, this.value);
            }
        }, false);
        function rotationVector(k) {
            if(k != rotationVectorV) {
                rotationVectorV = k;
                sendRequest(2, k);
            }
        }
        function changeMode(k) {
            if(k != activeMode) {
                activeMode = k;
                sendRequest(3, k);
            }
        }

        function getspeed() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                    document.querySelector("#speedValue").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "getspeed", true);
            xhttp.send();
        }

        function getgramms() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                    document.querySelector("#gramms").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "getgramms", true);
            xhttp.send();
        }

        function sendRequest(block, details){
            /*
            power - 0 - 0, 1 > send
            speedRange - 1 - 0-100 int > send
            rotationVector - 2 - 0, 1 > send
                | 1 - аверс, 0 - реверс
            modes - 3 - 1-n > send
            */
            var xhr;
            xhr = new XMLHttpRequest();
            xhr.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                    //document.getElementById("state").innerHTML = this.responseText;
                }
            };
            xhr.open("GET", "control?block="+block+"&details="+details, true);
            xhr.send();
        }
    </script>        
</body>
</html>
)rawliteral";
