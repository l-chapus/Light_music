package com.example.bragi

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.Intent
import android.content.pm.PackageManager
import android.util.Log
import android.os.Bundle
import android.os.Build
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.getValue
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import com.example.bragi.ui.theme.BragiTheme
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.IOException
import java.io.OutputStream
import java.util.UUID
import androidx.compose.foundation.text.KeyboardOptions


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

    // --- NOUVEAU : État de l'UI pour Compose ---
    private val _connectionStatus = mutableStateOf("Déconnecté")
    private val connectionStatus: State<String> get() = _connectionStatus

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        checkAndRequestBluetoothPermissions()
        enableEdgeToEdge()
        setContent {
            BragiTheme {
                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    // On passe l'état et les fonctions à notre UI
                    BluetoothControlScreen(
                        modifier = Modifier.padding(innerPadding),
                        connectionStatus = connectionStatus.value, // Passer l'état actuel
                        onConnectClick = { address -> // La fonction reçoit maintenant l'adresse
                            if (bluetoothAdapter?.isEnabled == false) {
                                val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
                                requestEnableBluetooth.launch(enableBtIntent)
                            } else {
                                connectToDevice(address)
                            }
                        },
                        onSendCommandClick = { command -> sendCommand(command) }
                    )
                }
            }
        }
    }

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
                withContext(Dispatchers.Main) { _connectionStatus.value = "Permission manquante" }
                return@launch
            }

            withContext(Dispatchers.Main) { _connectionStatus.value = "Connexion en cours..." }

            try {
                val device: BluetoothDevice? = bluetoothAdapter?.getRemoteDevice(deviceAddress)
                bluetoothSocket = device?.createRfcommSocketToServiceRecord(sppUuid)
                bluetoothAdapter?.cancelDiscovery()
                bluetoothSocket?.connect()
                outputStream = bluetoothSocket?.outputStream
                withContext(Dispatchers.Main) { _connectionStatus.value = "Connecté" }
                Log.d("BluetoothConnection", "Connecté avec succès à $deviceAddress")
            } catch (e: Exception) { // Utiliser Exception pour attraper toutes les erreurs possibles
                Log.e("BluetoothConnection", "Erreur de connexion: ${e.message}", e)
                try { bluetoothSocket?.close() } catch (ex: IOException) { }
                withContext(Dispatchers.Main) { _connectionStatus.value = "Erreur de connexion" }
            }
        }
    }

    private fun sendCommand(command: String) {
        if (outputStream == null) {
            Log.e("BluetoothCommand", "Non connecté. Impossible d'envoyer la commande.")
            return
        }
        CoroutineScope(Dispatchers.IO).launch {
            try {
                outputStream?.write(command.toByteArray())
                Log.d("BluetoothCommand", "Commande envoyée: $command")
            } catch (e: IOException) {
                Log.e("BluetoothCommand", "Erreur d'envoi: ${e.message}", e)
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        try {
            outputStream?.close()
            bluetoothSocket?.close()
        } catch (e: IOException) {
            Log.e("BluetoothCleanup", "Erreur lors de la fermeture.", e)
        }
    }
}

// --- INTERFACE UTILISATEUR AMÉLIORÉE AVEC CHAMPS RGB ---
@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun BluetoothControlScreen(
    modifier: Modifier = Modifier,
    connectionStatus: String,
    onConnectClick: (String) -> Unit,
    onSendCommandClick: (String) -> Unit
) {
    // État pour l'adresse MAC (inchangé)
    var macAddress by rememberSaveable { mutableStateOf("2C:BC:BB:A8:E2:7A") }

    // --- GESTION DE LA LISTE DÉROULANTE (inchangé) ---
    val commandList = listOf("Automatique", "Afficher_pixel", "Afficher_ligne", "Afficher_colonne")
    var isExpanded by remember { mutableStateOf(false) }
    var selectedCommand by remember { mutableStateOf(commandList[0]) }

    // --- NOUVEAU : GESTION DES CHAMPS RGB ---
    var redValue by rememberSaveable { mutableStateOf("0") }
    var greenValue by rememberSaveable { mutableStateOf("0") }
    var blueValue by rememberSaveable { mutableStateOf("0") }
    var vitesse by rememberSaveable { mutableStateOf("0") }
    // --- FIN DE LA NOUVELLE GESTION ---

    Column(
        modifier = modifier
            .fillMaxSize()
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "Bragi Bluetooth Control",
            fontSize = 24.sp,
            fontWeight = FontWeight.Bold
        )

        // Champ de texte pour l'adresse MAC (inchangé)
        OutlinedTextField(
            value = macAddress,
            onValueChange = { macAddress = it },
            label = { Text("Adresse MAC de l'appareil") },
            modifier = Modifier.fillMaxWidth()
        )

        // Statut de la connexion (inchangé)
        Text(
            text = "Statut : $connectionStatus",
            color = when (connectionStatus) {
                "Connecté" -> Color(0xFF4CAF50)
                "Erreur de connexion", "Permission manquante", "Adresse MAC invalide" -> MaterialTheme.colorScheme.error
                else -> Color.Gray
            },
            fontWeight = FontWeight.SemiBold
        )

        // Bouton de connexion (inchangé)
        Button(
            onClick = { onConnectClick(macAddress) },
            modifier = Modifier.fillMaxWidth()
        ) {
            Text("Se Connecter")
        }

        // --- NOUVELLE SECTION DE COMMANDE (juste après la connexion) ---
        // Conteneur pour la liste déroulante (inchangé)
        ExposedDropdownMenuBox(
            expanded = isExpanded,
            onExpandedChange = { isExpanded = it },
            modifier = Modifier.fillMaxWidth()
        ) {
            OutlinedTextField(
                value = selectedCommand,
                onValueChange = {},
                readOnly = true,
                label = { Text("Choisir une commande") },
                trailingIcon = { ExposedDropdownMenuDefaults.TrailingIcon(expanded = isExpanded) },
                modifier = Modifier
                    .menuAnchor()
                    .fillMaxWidth()
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

        // --- RANGÉE POUR LES CHAMPS RGB (MISE À JOUR) ---
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            if (selectedCommand == "Afficher_pixel" || selectedCommand == "Afficher_ligne") {// Champ pour la valeur Rouge
                OutlinedTextField(
                    value = redValue,
                    onValueChange = { redValue = it.filter { char -> char.isDigit() }.take(3) },
                    label = { Text("Red") },
                    modifier = Modifier.weight(1f),
                    // --- AJOUT ---
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number)
                )
                // Champ pour la valeur Verte
                OutlinedTextField(
                    value = greenValue,
                    onValueChange = { greenValue = it.filter { char -> char.isDigit() }.take(3) },
                    label = { Text("Green") },
                    modifier = Modifier.weight(1f),
                    // --- AJOUT ---
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number)
                )
                // Champ pour la valeur Bleue
                OutlinedTextField(
                    value = blueValue,
                    onValueChange = { blueValue = it.filter { char -> char.isDigit() }.take(3) },
                    label = { Text("Blue") },
                    modifier = Modifier.weight(1f),
                    // --- AJOUT ---
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number)
                )
            }
            if (selectedCommand == "FLASH") {// Champ pour la valeur Rouge
                OutlinedTextField(
                    value = vitesse,
                    onValueChange = { vitesse = it.filter { char -> char.isDigit() }.take(3) },
                    label = { Text("Temps") },
                    modifier = Modifier.weight(1f),
                    // --- AJOUT ---
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number)
                )
            }
        }


        // Le Spacer est maintenant ici pour pousser le bouton d'envoi vers le bas
        Spacer(modifier = Modifier.weight(1f))

        // Bouton unique pour envoyer la commande finale
        Button(
            onClick = {
                // Construit la commande finale en concaténant les valeurs RGB
                val finalCommand = "$selectedCommand:${redValue.ifBlank { "0" }}:${greenValue.ifBlank { "0" }}:${blueValue.ifBlank { "0" }}"
                onSendCommandClick(finalCommand)
            },
            enabled = connectionStatus == "Connecté",
            modifier = Modifier.fillMaxWidth()
        ) {
            Text("Envoyer la Commande")
        }
    }
}

@Preview(showBackground = true)
@Composable
fun DefaultPreview() {
    BragiTheme {
        BluetoothControlScreen(
            connectionStatus = "Déconnecté",
            onConnectClick = {},
            onSendCommandClick = {}
        )
    }
}