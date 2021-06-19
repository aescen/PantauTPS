 <?php
$servername = "localhost";
$username = "root";
$password = "";
$dbname = "wsn";

// Create connection
$conn = new mysqli($servername, $username, $password, $dbname);
// Check connection
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
}

$node = $_POST['node'];
$s1 = $_POST['temp'];
$s2 = $_POST['RH'];
$s3 = $_POST['moist'];

$sql = "UPDATE `Status` SET `Temperature` = $s1, `Humidity` = $s2, `Moisture` = $s3 WHERE `Status`.`Node` = $node";

if ($conn->query($sql) === TRUE) {
    echo "Record updated successfully";
} else {
    echo "Error updating record: " . $conn->error;
}

$conn->close();
?> 
