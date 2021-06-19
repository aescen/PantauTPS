-- phpMyAdmin SQL Dump
-- version 5.0.2
-- https://www.phpmyadmin.net/
--
-- Host: 127.0.0.1
-- Generation Time: May 31, 2021 at 11:32 AM
-- Server version: 10.4.13-MariaDB
-- PHP Version: 7.4.7

SET SQL_MODE = "NO_AUTO_VALUE_ON_ZERO";
START TRANSACTION;
SET time_zone = "+00:00";


/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;

--
-- Database: `pi`
--

-- --------------------------------------------------------

--
-- Table structure for table `pantautps`
--

DROP TABLE IF EXISTS `pantautps`;
CREATE TABLE `pantautps` (
  `Id` tinyint(3) UNSIGNED NOT NULL,
  `Nama` varchar(4) COLLATE utf8mb4_unicode_ci NOT NULL,
  `Nilai` float NOT NULL,
  `Waktu` timestamp NOT NULL DEFAULT current_timestamp() ON UPDATE current_timestamp(),
  `Node` tinyint(3) UNSIGNED NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;

--
-- Dumping data for table `pantautps`
--

INSERT INTO `pantautps` (`Id`, `Nama`, `Nilai`, `Waktu`, `Node`) VALUES
(1, 'ch4', 111, '2021-05-23 08:17:53', 1),
(2, 'co2', 222, '2021-05-23 08:18:12', 1),
(3, 'nh3', 333, '2021-05-23 08:18:03', 1),
(4, 'ch4', 44, '2021-05-23 07:07:23', 2),
(5, 'nh3', 55, '2021-05-23 07:07:26', 2),
(6, 'co2', 66, '2021-05-23 07:07:30', 2),
(7, 'ch4', 99, '2021-05-23 07:07:38', 3),
(8, 'nh3', 88, '2021-05-23 07:07:36', 3),
(9, 'co2', 77, '2021-05-23 07:07:33', 3),
(10, 'ch4', 101, '2021-05-23 07:07:53', 4),
(11, 'co2', 111, '2021-05-23 07:07:58', 4),
(12, 'nh3', 121, '2021-05-23 07:08:02', 4);

--
-- Triggers `pantautps`
--
DROP TRIGGER IF EXISTS `pantautps_logging`;
DELIMITER $$
CREATE TRIGGER `pantautps_logging` AFTER UPDATE ON `pantautps` FOR EACH ROW REPLACE INTO pantautpslog(Id, Nama, Nilai, Waktu, Node)
        VALUES(NULL, new.Nama, new.Nilai, CURRENT_TIMESTAMP, new.Node)
$$
DELIMITER ;

-- --------------------------------------------------------

--
-- Table structure for table `pantautpslog`
--

DROP TABLE IF EXISTS `pantautpslog`;
CREATE TABLE `pantautpslog` (
  `Id` int(11) UNSIGNED NOT NULL,
  `Nama` varchar(4) NOT NULL,
  `Nilai` float NOT NULL,
  `Waktu` timestamp NOT NULL DEFAULT current_timestamp() ON UPDATE current_timestamp(),
  `Node` tinyint(4) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

--
-- Dumping data for table `pantautpslog`
--

INSERT INTO `pantautpslog` (`Id`, `Nama`, `Nilai`, `Waktu`, `Node`) VALUES
(1, 'ch4', 1, '2021-05-23 07:04:56', 1),
(2, 'co2', 2, '2021-05-23 07:05:01', 1),
(3, 'nh3', 3, '2021-05-23 07:05:07', 1),
(4, 'ch4', 4, '2021-05-23 07:05:12', 2),
(5, 'nh3', 5, '2021-05-23 07:05:15', 2),
(6, 'co2', 6, '2021-05-23 07:05:19', 2),
(7, 'co2', 7, '2021-05-23 07:05:24', 3),
(8, 'nh3', 8, '2021-05-23 07:05:29', 3),
(9, 'ch4', 9, '2021-05-23 07:05:33', 3),
(10, 'ch4', 10, '2021-05-23 07:05:37', 4),
(11, 'co2', 11, '2021-05-23 07:05:41', 4),
(12, 'nh3', 12, '2021-05-23 07:05:45', 4),
(13, 'ch4', 11, '2021-05-23 07:06:59', 1),
(14, 'co2', 22, '2021-05-23 07:07:04', 1),
(15, 'nh3', 33, '2021-05-23 07:07:08', 1),
(16, 'ch4', 44, '2021-05-23 07:07:23', 2),
(17, 'nh3', 55, '2021-05-23 07:07:26', 2),
(18, 'co2', 66, '2021-05-23 07:07:30', 2),
(19, 'co2', 77, '2021-05-23 07:07:33', 3),
(20, 'nh3', 88, '2021-05-23 07:07:36', 3),
(21, 'ch4', 99, '2021-05-23 07:07:38', 3),
(22, 'ch4', 101, '2021-05-23 07:07:53', 4),
(23, 'co2', 111, '2021-05-23 07:07:58', 4),
(24, 'nh3', 121, '2021-05-23 07:08:02', 4),
(25, 'ch4', 111, '2021-05-23 08:17:53', 1),
(26, 'nh3', 333, '2021-05-23 08:18:03', 1),
(27, 'co2', 222, '2021-05-23 08:18:12', 1);

--
-- Indexes for dumped tables
--

--
-- Indexes for table `pantautps`
--
ALTER TABLE `pantautps`
  ADD PRIMARY KEY (`Id`);

--
-- Indexes for table `pantautpslog`
--
ALTER TABLE `pantautpslog`
  ADD PRIMARY KEY (`Id`);

--
-- AUTO_INCREMENT for dumped tables
--

--
-- AUTO_INCREMENT for table `pantautpslog`
--
ALTER TABLE `pantautpslog`
  MODIFY `Id` int(11) UNSIGNED NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=28;
COMMIT;

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
