<!-- PHP.MySQL.START -->
<?php
    @session_start();
	
	$nodeId = 0;
    
    if(isset($_POST['resetDate'])){
        unset($_SESSION['dateselect']);
        unset($_POST['dateselect']);
        unset($dateSelect);
        unset($_SESSION['ymd']);
    }
    if(isset($_POST['dateselect']) && isset($_POST['nodeid'])){
        $_SESSION['dateselect'] = $_POST['dateselect'];
        $_SESSION['nodeid'] = $_POST['nodeid'];
    }
    
    if(isset($_SESSION['dateselect']) && isset($_SESSION['nodeid'])){
        $dateSelect = $_SESSION['dateselect'];
        $nodeId = $_SESSION['nodeid'];
    }
    else{
        unset($dateSelect);
        unset($resultsHistory);
    }
    
    require_once 'paginator.php';
    require_once 'config.php';
    
    $connHistory       = $conn;    
 
    $limitHistory      = ( isset( $_GET['limit'] ) ) ? $_GET['limit'] : 25;
    $pageHistory       = ( isset( $_GET['page'] ) ) ? $_GET['page'] : 1;
    $linksHistory      = ( isset( $_GET['links'] ) ) ? $_GET['links'] : 7;
    if( isset($dateSelect) ){
		$startTime = $dateSelect . " 00:00:00.000000";
        $endTime = $dateSelect . " 23:59:59.999999";
		if ($nodeId != 0){
			$queryHistory      = "SELECT * FROM `pantautpslog` WHERE `Node` = $nodeId AND `Waktu` BETWEEN '$startTime' AND '$endTime' ORDER BY `id` DESC";
		} else {
			$queryHistory      = "SELECT * FROM `pantautpslog` WHERE `Waktu` BETWEEN '$startTime' AND '$endTime' ORDER BY `id` DESC";
		}
        
        $ymd = date("Y-m-d", strtotime($dateSelect));
        $_SESSION['ymd'] = $ymd;
        
    } else{
        $queryHistory      = "SELECT * FROM `pantautpslog`";
    }
    $PaginatorHistory  = new Paginator( $connHistory, $queryHistory);
    if ($PaginatorHistory->getNumRows() > 0){
        $resultsHistory    = $PaginatorHistory->getData( $pageHistory, $limitHistory );
    }
?>
<!DOCTYPE html>
<html>
    <head>
		<title>MONITORING TEMPAT PEMBUANGAN SAMPAH</title>
		<meta charset="UTF-8">
		<meta name="viewport" content="width=device-width, initial-scale=1">
		<link rel="shortcut icon" type="image/png" href="static/images/favicon.png"/>
		<link rel="stylesheet" href="static/styles/w3.css">
		<link rel="stylesheet" href="static/styles/css.css">
		<link rel="stylesheet" href="static/styles/font-awesome.css">
		<script src="static/scripts/canvasjs.min.js"></script>
		<script src="static/scripts/jquery.min.js"></script>
        <style>
            html,body,h1,h2,h3,h4,h5,h6 {font-family: "Raleway", sans-serif}
        </style>
    </head>
    <body class="w3-light-grey" onload="updateClock()">
        <!-- Top container -->
        <div class="w3-bar w3-top w3-black w3-large" style="z-index:4">
		  <button type="button" class="w3-bar-item w3-button w3-hide-large w3-hover-none w3-hover-text-light-grey" onclick="w3_open();"><i class="fa fa-bars"></i>  Menu</button>
		  <span class="w3-bar-item w3-xlarge fa fa-cubes"> Monitoring Tempat Pembuangan Sampah</span>
		</div>

        <!-- Sidebar/menu -->
		<nav class="w3-sidebar w3-collapse w3-white w3-animate-left" style="z-index:3;width:300px;" id="mySidebar">
		  <!-- <div class="w3-container w3-row">
			<div class="w3-col s4">
			  <img src="static/images/avatar2.png" class="w3-circle w3-margin-center" style="width:46px">
			</div>
			<div class="w3-col s8 w3-bar">
			  <span>Welcome, <strong>user</strong></span><br>
			  <a href="#" class="w3-bar-item w3-button"><i class="fa fa-envelope"></i></a>
			  <a href="#" class="w3-bar-item w3-button"><i class="fa fa-user"></i></a>
			  <a href="#" class="w3-bar-item w3-button"><i class="fa fa-cog"></i></a>
			</div>
		  </div> -->
		  <div class="w3-container w3-black">
			<h5>Dashboard</h5>
		  </div>
		  <div class="w3-bar-block">
			<a href="#" class="w3-bar-item w3-button w3-padding-16 w3-hide-large w3-grey w3-hover-black" onclick="w3_close()" title="close menu"><i class="fa fa-remove fa-fw"></i>  Close Menu</a>
            <a href="index.php" class="w3-bar-item w3-button w3-padding"><i class="fa fa-dashboard fa-fw"></i>  Node 1</a>
		    <a href="node2.php" class="w3-bar-item w3-button w3-padding"><i class="fa fa-dashboard fa-fw"></i>  Node 2</a>
		    <a href="node3.php" class="w3-bar-item w3-button w3-padding"><i class="fa fa-dashboard fa-fw"></i>  Node 3</a>
		    <a href="node4.php" class="w3-bar-item w3-button w3-padding"><i class="fa fa-dashboard fa-fw"></i>  Node 4</a>
			<a href="logs.php" class="w3-bar-item w3-button w3-padding w3-blue"><i class="fa fa-dashboard fa-fw"></i>  Logs</a>
          </div>
        </nav>

        <!-- Overlay effect when opening sidebar on small screens -->
        <div class="w3-overlay w3-hide-large w3-animate-opacity" onclick="w3_close()" style="cursor:pointer" title="close side menu" id="myOverlay"></div>

        <!-- !PAGE CONTENT! -->
        <div class="w3-main" style="margin-left:300px;margin-top:43px;">

          <!-- Header -->
          <header class="w3-container" style="padding-top:22px">
             <div class="w3-cell-row w3-blue-grey">
              <div class="w3-container w3-cell">
                <h2>Logs <?php echo ($nodeId == 0)? '(All Node)': '(Node ' . $nodeId . ')'; ?> </h2>
              </div>
              <div class="w3-container w3-cell w3-right-align">
                <h2 id="clock">Clock</h2>
              </div>
            </div>
          </header>
          
          <!-- Feeds panel -->
          <div class="w3-panel">
            <div class="w3-row-padding w3-container" style="margin:0 -16px">
              <div>
                <!-- PUT TABLE HERE -->                
                <div class="w3-cell-row">
                    <div class="w3-left-align">
                        <form action="logs.php" method="post" onsubmit="return valDate()" required>
                            <label for="nodeid">Node</label>
                            <select id="nodeid" name="nodeid" autofocus>
                             <option value="0" <?php echo ($nodeId == 0)? 'selected': ''; ?> >All</option>
                             <option value="1" <?php echo ($nodeId == 1)? 'selected': ''; ?> >Node 1</option>
                             <option value="2" <?php echo ($nodeId == 2)? 'selected': ''; ?> >Node 2</option>
                             <option value="3" <?php echo ($nodeId == 3)? 'selected': ''; ?> >Node 3</option>
                             <option value="4" <?php echo ($nodeId == 4)? 'selected': ''; ?> >Node 4</option>
                            </select>
                            <label for="dateselect">Set tanggal: </label>
                            <input type="date" id="dateselect" name="dateselect" min="2019-01-01" max="<?php echo date("Y-m-d"); ?>" value="<?php echo (isset($_SESSION['ymd']) ? $_SESSION['ymd'] : ""); ?>">
                            <input type="submit" value="Pilih">
                            <input type="submit" value="Reset" id="resetDate" name="resetDate">
                        </form>
                    </div>
                </div>
                <br>
                <table class="w3-table-all w3-centered w3-card-4 w3-white">
                            <tr class='w3-blue-grey'>
                                <th style="vertical-align: middle;">No</th>
                                <th style="vertical-align: middle;">Nama Gas</th>
                                <th style="vertical-align: middle;">Nilai Sensor</th>
                                <th style="vertical-align: middle;">Waktu</th>
                            </tr>
                            <?php    
                                    if ($PaginatorHistory->getNumRows() > 0) {
                                        $validData = 0;
                                        for( $i = 0; $i < count( $resultsHistory->data ); $i++ ) :
                                            echo "<tr>";
                                                echo "<td>" . ($i+1) . "</td>";
                                                echo "<td>" . strtoupper($resultsHistory->data[$i]['Nama']) . "</td>";
                                                echo "<td>" . $resultsHistory->data[$i]['Nilai'] . "</td>";
                                                echo "<td>" . $resultsHistory->data[$i]['Waktu'] . "</td>";
                                            echo "</tr>";
                                        endfor;
                                    }
                                    else{
                                        echo "<tr><td colspan='4'>Kosong</td></tr>";
                                    }
                            ?>
                </table>
                <br>
                <?php
                    if ($PaginatorHistory->getNumRows() > 0){
                        if ($PaginatorHistory->getLastPageNum() > 1){
                            echo $PaginatorHistory->createLinks( $linksHistory, 'w3-bar-item w3-button w3-border' );
                        }
                    }
                ?>
              </div>
            </div>
          </div>
          <!-- End page content -->
        </div>
        <script>
            // Get the Sidebar
            var mySidebar = document.getElementById("mySidebar");

            // Get the DIV with overlay effect
            var overlayBg = document.getElementById("myOverlay");

            // Toggle between showing and hiding the sidebar, and add overlay effect
            function w3_open() {
              if (mySidebar.style.display === 'block') {
                mySidebar.style.display = 'none';
                overlayBg.style.display = "none";
              } else {
                mySidebar.style.display = 'block';
                overlayBg.style.display = "block";
              }
            }

            // Close the sidebar with the close button
            function w3_close() {
              mySidebar.style.display = "none";
              overlayBg.style.display = "none";
            }
            
            function valDate(){
                var valueDate = document.getElementById('dateselect').value;
                if(!Date.parse(valueDate)){
                    alert('Date is invalid or empty!');
                    return false;
                }
            }
        </script>
        <script type="text/javascript">
            function updateClock ( ){
                var currentTime = new Date ();

                var currentYear = currentTime.getFullYear();
                var currentMonth = currentTime.getMonth();
                var currentDate = currentTime.getDate();
                var currentHours = currentTime.getHours ( );
                var currentMinutes = currentTime.getMinutes ( );
                var currentSeconds = currentTime.getSeconds ( );

                const monthNames = ["Jan", "Feb", "Mar", "Apr", "Mei", "Jun",
                                      "Jul", "Agu", "Sep", "Okt", "Nov", "Des"
                                    ];

                // Pad the minutes and seconds with leading zeros, if required
                currentMinutes = ( currentMinutes < 10 ? "0" : "" ) + currentMinutes;
                currentSeconds = ( currentSeconds < 10 ? "0" : "" ) + currentSeconds;

                // Choose either "AM" or "PM" as appropriate
                //var timeOfDay = ( currentHours < 12 ) ? "AM" : "PM";

                // Convert the hours component to 12-hour format if needed
                //currentHours = ( currentHours > 12 ) ? currentHours - 12 : currentHours;

                // Convert an hours component of "0" to "12"
                //currentHours = ( currentHours == 0 ) ? 12 : currentHours;

                // Compose the string for display
                //var currentTimeString = currentHours + ":" + currentMinutes + ":" + currentSeconds + " " + timeOfDay;
                var currentTimeString =  currentDate + " " + monthNames[currentMonth] + " "+ currentYear + " " + currentHours + ":" + currentMinutes + ":" + currentSeconds;

                // Update the time display
                document.getElementById("clock").firstChild.nodeValue = currentTimeString;
            }
            setInterval('updateClock()', 1000 );
            
            function valBatch(){
                var valueBatch = document.getElementById('batchselect').value;
                if((valueBatch == '')){
                    alert('Text is invalid or empty!');
                    return false;
                }
            }
            
        </script>
    </body>
</html>