package com.example.solnir

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter // CORRECTION : Import manquant
import android.content.pm.PackageManager
import android.util.Log
import android.os.Bundle
import android.os.Build
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.getValue
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import com.example.solnir.ui.theme.SolnirTheme
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.IOException
import java.io.OutputStream
import java.util.UUID
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.tooling.preview.Preview


class MainActivity : ComponentActivity() {

    // --- Permissions et activation Bluetooth (inchangé) ---
    private val requestBluetoothPermissions = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
        permissions.entries.forEach { Log.d("BluetoothPermissions", "${it.key} = ${it.value}") }
    }
    private val requestEnableBluetooth = registerForActivityResult(ActivityResultContracts.StartActivityForResult()) { result ->
        if (result.resultCode == RESULT_OK) Log.d("BluetoothState", "Bluetooth activé.")
        else Log.d("BluetoothState", "Activation du Bluetooth refusée.")
    }

    // --- Gestionnaires Bluetooth (inchangé) ---
    private val bluetoothManager: BluetoothManager by lazy { getSystemService(BluetoothManager::class.java) }
    private val bluetoothAdapter: BluetoothAdapter? by lazy { bluetoothManager.adapter }

    // --- Socket et communication (inchangé) ---
    private var bluetoothSocket: BluetoothSocket? = null
    private var outputStream: OutputStream? = null
    private val sppUuid: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

    // --- État de l'UI pour Compose ---
    private val _connectionStatus = mutableStateOf("Déconnecté")
    private val connectionStatus: State<String> get() = _connectionStatus

    private val _scannedDevices = mutableStateOf<List<BluetoothDevice>>(emptyList())
    val scannedDevices: State<List<BluetoothDevice>> get() = _scannedDevices

    private val _isScanning = mutableStateOf(false)
    val isScanning: State<Boolean> get() = _isScanning

    // --- BroadcastReceiver pour le scan ---
    private val receiver = object : BroadcastReceiver() {
        @SuppressLint("MissingPermission") // CORRECTION : Annotation placée sur la méthode pour couvrir l'accès à `it.name`
        override fun onReceive(context: Context, intent: Intent) {
            val action: String? = intent.action
            when (action) {
                BluetoothDevice.ACTION_FOUND -> {
                    val device: BluetoothDevice? = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                        intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java)
                    } else {
                        @Suppress("DEPRECATION")
                        intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                    }

                    device?.let {
                        // On vérifie que l'appareil a un nom et qu'il n'est pas déjà dans la liste
                        if (it.name != null && !_scannedDevices.value.any { d -> d.address == it.address }) {
                            _scannedDevices.value = _scannedDevices.value + it
                        }
                    }
                }
                BluetoothAdapter.ACTION_DISCOVERY_FINISHED -> {
                    _isScanning.value = false
                    Log.d("BluetoothScan", "Scan terminé.")
                }
            }
        }
    }

    // --- Fonctions pour le scan ---
    @SuppressLint("MissingPermission")
    private fun startScan() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
            Log.e("BluetoothScan", "La permission BLUETOOTH_SCAN est manquante.")
            return
        }

        if (bluetoothAdapter?.isEnabled == false) {
            val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            requestEnableBluetooth.launch(enableBtIntent)
            return
        }

        // Arrêter un scan précédent avant d'en lancer un nouveau
        stopScan()

        _scannedDevices.value = emptyList()
        _isScanning.value = true

        // Enregistrer le receiver AVANT de démarrer le scan
        val filter = IntentFilter(BluetoothDevice.ACTION_FOUND)
        filter.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
        registerReceiver(receiver, filter)

        bluetoothAdapter?.startDiscovery()
        Log.d("BluetoothScan", "Scan démarré...")
    }

    @SuppressLint("MissingPermission")
    private fun stopScan() {
        if (bluetoothAdapter?.isDiscovering == true) {
            bluetoothAdapter?.cancelDiscovery()
        }
        _isScanning.value = false
        try {
            unregisterReceiver(receiver)
        } catch (e: IllegalArgumentException) {
            Log.w("BluetoothScan", "Receiver non enregistré, pas besoin de le désenregistrer.")
        }
    }

    // --- GESTION DU CYCLE DE VIE ---
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        checkAndRequestBluetoothPermissions()
        enableEdgeToEdge()
        setContent {
            SolnirTheme {
                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    BluetoothControlScreen(
                        modifier = Modifier.padding(innerPadding),
                        connectionStatus = connectionStatus.value,
                        scannedDevices = scannedDevices.value,
                        isScanning = isScanning.value,
                        onConnectClick = { address ->
                            stopScan()
                            connectToDevice(address)
                        },
                        onSendCommandClick = { command -> sendCommand(command) },
                        onStartScanClick = { startScan() }
                    )
                }
            }
        }
    }

    // CORRECTION : Nettoyage dans onDestroy
    override fun onDestroy() {
        super.onDestroy()
        stopScan() // Très important pour éviter les fuites de mémoire
        try {
            outputStream?.close()
            bluetoothSocket?.close()
        } catch (e: IOException) {
            Log.e("BluetoothCleanup", "Erreur lors de la fermeture des sockets.", e)
        }
    }

    // --- Logique de connexion et permissions (majoritairement inchangée) ---
    private fun checkAndRequestBluetoothPermissions() {
        val requiredPermissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }
        val missingPermissions = requiredPermissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }
        if (missingPermissions.isNotEmpty()) {
            requestBluetoothPermissions.launch(missingPermissions.toTypedArray())
        }
    }

    private fun connectToDevice(deviceAddress: String) {
        if (deviceAddress.isBlank() || !BluetoothAdapter.checkBluetoothAddress(deviceAddress)) {
            _connectionStatus.value = "Adresse MAC invalide"
            return
        }

        CoroutineScope(Dispatchers.IO).launch {
            if (ContextCompat.checkSelfPermission(this@MainActivity, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                withContext(Dispatchers.Main) { _connectionStatus.value = "Permission de connexion manquante" }
                return@launch
            }

            withContext(Dispatchers.Main) { _connectionStatus.value = "Connexion en cours..." }

            try {
                val device: BluetoothDevice? = bluetoothAdapter?.getRemoteDevice(deviceAddress)
                bluetoothSocket = device?.createRfcommSocketToServiceRecord(sppUuid)
                stopScan() // Sécurité : on arrête le scan avant de se connecter
                bluetoothSocket?.connect()
                outputStream = bluetoothSocket?.outputStream
                withContext(Dispatchers.Main) { _connectionStatus.value = "Connecté" }
                Log.d("BluetoothConnection", "Connecté avec succès à $deviceAddress")
            } catch (e: Exception) {
                Log.e("BluetoothConnection", "Erreur de connexion: ${e.message}", e)
                try { bluetoothSocket?.close() } catch (ex: IOException) { }
                withContext(Dispatchers.Main) { _connectionStatus.value = "Erreur de connexion" }
            }
        }
    }

    private fun sendCommand(command: String) {
        if (outputStream == null) {
            Log.e("BluetoothCommand", "Non connecté. Impossible d'envoyer la commande.")
            _connectionStatus.value = "Déconnecté" // Mettre à jour l'UI si on essaie d'envoyer sans être connecté
            return
        }
        CoroutineScope(Dispatchers.IO).launch {
            try {
                outputStream?.write(command.toByteArray())
                Log.d("BluetoothCommand", "Commande envoyée: $command")
            } catch (e: IOException) {
                Log.e("BluetoothCommand", "Erreur d'envoi: ${e.message}", e)
                withContext(Dispatchers.Main) {
                    _connectionStatus.value = "Erreur de connexion"
                }
            }
        }
    }
}


// --- INTERFACE UTILISATEUR (avec la fin du fichier corrigée) ---
@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun BluetoothControlScreen(
    modifier: Modifier = Modifier,
    connectionStatus: String,
    scannedDevices: List<BluetoothDevice>,
    isScanning: Boolean,
    onConnectClick: (String) -> Unit,
    onSendCommandClick: (String) -> Unit,
    onStartScanClick: () -> Unit
) {
    var macAddress by rememberSaveable { mutableStateOf("2C:BC:BB:A8:E2:7A") }
    val commandList = listOf("Automatique", "Afficher_pixel", "Afficher_colonne", "Afficher_ligne", "Couleur_statique", "Effect_trou_noir", "Defilement_vague", "Defilement_RBG", "Stroboscope", "Bruit_television", "Respiration", "Couleur_aleatoire", "Matrix", "Halo", "Défilement_sens", "Change_id", "Stop_Led", "Batterie_niveau", "Afficher_batterie")
    var isExpanded by remember { mutableStateOf(false) }
    var selectedCommand by remember { mutableStateOf(commandList[0]) }
    var redValue by rememberSaveable { mutableStateOf("0") }
    var greenValue by rememberSaveable { mutableStateOf("0") }
    var blueValue by rememberSaveable { mutableStateOf("0") }
    var vitesse by rememberSaveable { mutableStateOf("0") }
    var sens by rememberSaveable { mutableStateOf("0") }
    var numeroLigne by rememberSaveable { mutableStateOf("0") }
    var numeroColonne by rememberSaveable { mutableStateOf("0") }
    var positionX by rememberSaveable { mutableStateOf("0") }
    var positionY by rememberSaveable { mutableStateOf("0") }
    var Id by rememberSaveable { mutableStateOf("1") }

    Column(
        modifier = modifier
            .fillMaxSize()
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "Solnir Bluetooth Control",
            fontSize = 24.sp,
            fontWeight = FontWeight.Bold
        )


        Divider(modifier = Modifier.padding(vertical = 8.dp))

        Button(
            onClick = onStartScanClick,
            enabled = !isScanning,
            modifier = Modifier.fillMaxWidth()
        ) {
            Text("Scanner les appareils")
        }

        if (isScanning) {
            CircularProgressIndicator(modifier = Modifier.padding(8.dp))
        }

        LazyColumn(
            modifier = Modifier
                .fillMaxWidth()
                .heightIn(max = 200.dp)
        ) {
            items(scannedDevices) { device ->
                @SuppressLint("MissingPermission")
                val deviceName = device.name ?: "Appareil inconnu"
                val deviceAddress = device.address

                Card(
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(vertical = 4.dp) // CORRECTION : Fin de la ligne
                        .clickable {
                            macAddress =
                                deviceAddress // Met à jour l'adresse MAC affichée (optionnel)
                            onConnectClick(deviceAddress) // Lance la connexion
                        },
                    elevation = CardDefaults.cardElevation(defaultElevation = 2.dp)
                ) {
                    Column(modifier = Modifier.padding(16.dp)) {
                        Text(text = deviceName, fontWeight = FontWeight.Bold)
                        Text(text = deviceAddress, style = MaterialTheme.typography.bodySmall)
                    }
                }
            }
        }

        Divider(modifier = Modifier.padding(vertical = 8.dp))

        Text(text = "Statut : $connectionStatus", color = if (connectionStatus == "Connecté") Color.Green else Color.Red)

        ExposedDropdownMenuBox(
            expanded = isExpanded,
            onExpandedChange = { isExpanded = it }
        ) {
            TextField(
                value = selectedCommand,
                onValueChange = {},
                readOnly = true,
                trailingIcon = { ExposedDropdownMenuDefaults.TrailingIcon(expanded = isExpanded) },
                modifier = Modifier
                    .menuAnchor()
                    .fillMaxWidth(),
                label = { Text("Commande") }
            )
            ExposedDropdownMenu(
                expanded = isExpanded,
                onDismissRequest = { isExpanded = false }
            ) {
                commandList.forEach { command ->
                    DropdownMenuItem(
                        text = { Text(command) },
                        onClick = {
                            selectedCommand = command
                            isExpanded = false
                        }
                    )
                }
            }
        }

        // Affiche les champs RGB si la commande n'est pas "Automatique"
        if (selectedCommand == "Afficher_pixel" || selectedCommand == "Afficher_ligne" || selectedCommand == "Afficher_colonne" || selectedCommand == "Couleur_statique" || selectedCommand == "Effect_trou_noir" || selectedCommand == "Defilement_vague") {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                val fieldModifier = Modifier.weight(1f)
                TextField(value = redValue, onValueChange = { redValue = it }, label = { Text("R") }, keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number), modifier = fieldModifier)
                TextField(value = greenValue, onValueChange = { greenValue = it }, label = { Text("G") }, keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number), modifier = fieldModifier)
                TextField(value = blueValue, onValueChange = { blueValue = it }, label = { Text("B") }, keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number), modifier = fieldModifier)
            }
        }

        // Affiche les champs de position X et Y uniquement pour "Afficher_pixel"
        if (selectedCommand == "Afficher_pixel") {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                val fieldModifier = Modifier.weight(1f)
                TextField(value = positionX, onValueChange = { positionX = it }, label = { Text("Position X") }, keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number), modifier = fieldModifier)
                TextField(value = positionY, onValueChange = { positionY = it }, label = { Text("Position Y") }, keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number), modifier = fieldModifier)
            }
        }
        if (selectedCommand == "Afficher_ligne") {
            // Affiche le champ Vitesse pour toutes les commandes
            TextField(value = numeroLigne, onValueChange = { numeroLigne = it }, label = { Text("Numéro de la ligne") }, keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number), modifier = Modifier.fillMaxWidth())
        }
        if (selectedCommand == "Afficher_colonne") {
        // Affiche le champ Vitesse pour toutes les commandes
        TextField(value = numeroColonne, onValueChange = { numeroColonne = it }, label = { Text("Numéro de la colonne") }, keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number), modifier = Modifier.fillMaxWidth())
        }

        if (selectedCommand == "Effect_trou_noir" || selectedCommand == "Defilement_vague") {
            // Affiche le champ Vitesse pour toutes les commandes
            TextField(value = vitesse, onValueChange = { vitesse = it }, label = { Text("Vitesse") }, keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number), modifier = Modifier.fillMaxWidth())
            TextField(value = sens, onValueChange = { sens = it }, label = { Text("Sens") }, keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number), modifier = Modifier.fillMaxWidth())
        }

        Spacer(modifier = Modifier.weight(1f))

        Button(
            onClick = {
                // Adapter la construction de la commande en fonction de la sélection
                val commandString = when (selectedCommand) {
                    "Afficher_pixel" -> "${Id.toIntOrNull() ?: 0};101;${positionX.toIntOrNull() ?: 0};${positionY.toIntOrNull() ?: 0};${redValue.toIntOrNull() ?: 0};${greenValue.toIntOrNull() ?: 0};${blueValue.toIntOrNull() ?: 0}"
                    "Afficher_colonne" -> "${Id.toIntOrNull() ?: 0};102;${redValue.toIntOrNull() ?: 0};${greenValue.toIntOrNull() ?: 0};${blueValue.toIntOrNull() ?: 0};${numeroColonne.toIntOrNull() ?: 0}"
                    "Afficher_ligne" -> "${Id.toIntOrNull() ?: 0};103;${redValue.toIntOrNull() ?: 0};${greenValue.toIntOrNull() ?: 0};${blueValue.toIntOrNull() ?: 0};${numeroLigne.toIntOrNull() ?: 0}"
                    "Couleur_statique" -> "${Id.toIntOrNull() ?: 0};104;${redValue.toIntOrNull() ?: 0};${greenValue.toIntOrNull() ?: 0};${blueValue.toIntOrNull() ?: 0}"
                    "Effet_trou_noir" -> "${Id.toIntOrNull() ?: 0};105;${redValue.toIntOrNull() ?: 0};${greenValue.toIntOrNull() ?: 0};${blueValue.toIntOrNull() ?: 0};${vitesse.toIntOrNull() ?: 0};${sens.toIntOrNull() ?: 0}"
                    "Defilement_vague" -> "${Id.toIntOrNull() ?: 0};106;${redValue.toIntOrNull() ?: 0};${greenValue.toIntOrNull() ?: 0};${blueValue.toIntOrNull() ?: 0};${vitesse.toIntOrNull() ?: 0};${sens.toIntOrNull() ?: 0}"

                    "Automatique" -> "$selectedCommand;${vitesse.toIntOrNull() ?: 0}"
                    else -> "$selectedCommand;${redValue.toIntOrNull() ?: 0};${greenValue.toIntOrNull() ?: 0};${blueValue.toIntOrNull() ?: 0};${vitesse.toIntOrNull() ?: 0}"
                }
                onSendCommandClick(commandString)
            },
            modifier = Modifier.fillMaxWidth()
        ) {
            Text("Envoyer la Commande")
        }
    }
}

// Preview (si vous en avez une)
@Preview(showBackground = true)
@Composable
fun DefaultPreview() {
    SolnirTheme {
        BluetoothControlScreen(
            connectionStatus = "Déconnecté",
            scannedDevices = emptyList(),
            isScanning = false,
            onConnectClick = {},
            onSendCommandClick = {},
            onStartScanClick = {}
        )
    }
}