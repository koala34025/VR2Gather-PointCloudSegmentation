﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using OrchestratorWrapping;

public enum State {
    Offline, Online, Logged, Config, Create, Join, Lobby, InGame
}

public class OrchestratorLogin : MonoBehaviour {

    private static OrchestratorLogin instance;

    public static OrchestratorLogin Instance { get { return instance; } }

    #region GUI Components

    public bool developerOptions = true;
    public bool usePresenter = false;
    private int kindAudio = 0;
    private int kindPresenter = 0;
    private int ntpSyncThreshold = 2; // Magic number to be defined (in seconds)

    [HideInInspector] public bool isMaster = false;
    [HideInInspector] public string userID = "";

    private State state = State.Offline;
    private bool orchestratorConnected = false;
    private bool userLogged = false;
    private bool joining = false;

    [SerializeField] private string orchestratorUrl = "";
    [SerializeField] private bool autoRetrieveOrchestratorDataOnConnect = true;

    [Header("Info")]
    [SerializeField] private Text statusText = null;
    [SerializeField] private Text userId = null;
    [SerializeField] private Text userName = null;
    [SerializeField] private Text orchURLText = null;
    [SerializeField] private Text nativeVerText = null;
    [SerializeField] private Text playerVerText = null;
    [SerializeField] private Text orchVerText = null;
    [SerializeField] private Text ntpText = null;

    [Header("Login")]
    [SerializeField] private InputField userNameLoginIF = null;
    [SerializeField] private InputField userPasswordLoginIF = null;
    [SerializeField] private InputField connectionURILoginIF = null;
    [SerializeField] private InputField exchangeNameLoginIF = null;
    [SerializeField] private Dropdown representationTypeLoginDropdown = null;

    [Header("Config")]
    [SerializeField] private GameObject tvmInfoGO = null;
    [SerializeField] private InputField connectionURIConfigIF = null;
    [SerializeField] private InputField exchangeNameConfigIF = null;
    [SerializeField] private Dropdown representationTypeConfigDropdown = null;

    [Header("Create")]
    [SerializeField] private InputField sessionNameIF = null;
    [SerializeField] private InputField sessionDescriptionIF = null;
    [SerializeField] private Dropdown scenarioIdDrop = null;
    [SerializeField] private Toggle presenterToggle = null;
    [SerializeField] private Toggle liveToggle = null;
    [SerializeField] private Toggle noAudioToggle = null;
    [SerializeField] private Toggle socketAudioToggle = null;
    [SerializeField] private Toggle dashAudioToggle = null;

    [Header("Join")]
    [SerializeField] private Dropdown sessionIdDrop = null;

    [Header("Lobby")]
    [SerializeField] private Text sessionNameText = null;
    [SerializeField] private Text sessionDescriptionText = null;
    [SerializeField] private Text scenarioIdText = null;
    [SerializeField] private Text sessionNumUsersText = null;

    [Header("Buttons")]
    [SerializeField] private Button connectButton = null;
    [SerializeField] private Button okButton = null;
    [SerializeField] private Button loginButton = null;
    [SerializeField] private Button configButton = null;
    [SerializeField] private Button saveConfigButton = null;
    [SerializeField] private Button exitConfigButton = null;
    [SerializeField] private Button createButton = null;
    [SerializeField] private Button joinButton = null;
    [SerializeField] private Button doneCreateButton = null;
    [SerializeField] private Button doneJoinButton = null;
    [SerializeField] private Button readyButton = null;
    [SerializeField] private Button leaveButton = null;
    [SerializeField] private Button calibButton = null;
    [SerializeField] private Button refreshSessionsButton = null;

    [Header("Panels")]
    [SerializeField] private GameObject ntpPanel = null;
    [SerializeField] private GameObject usersButtonsPanel = null;
    [SerializeField] private GameObject loginPanel = null;
    [SerializeField] private GameObject configPanel = null;
    [SerializeField] private GameObject createPanel = null;
    [SerializeField] private GameObject joinPanel = null;
    [SerializeField] private GameObject lobbyPanel = null;
    [SerializeField] private GameObject sessionPanel = null;
    [SerializeField] private GameObject usersPanel = null;

    [Header("Content")]
    [SerializeField] private RectTransform orchestratorSessions = null;
    [SerializeField] private RectTransform usersSession = null;

    [Header("Logs container")]
    [SerializeField] private RectTransform logsContainer = null;
    [SerializeField] private ScrollRect logsScrollRect = null;

    #endregion

    #region Utils
    private Color onlineCol = new Color(0.15f, 0.78f, 0.15f); // Green
    private Color offlineCol = new Color(0.78f, 0.15f, 0.15f); // Red
    private Font ArialFont = null;
    private EventSystem system = null;
    #endregion

    #region GUI

    // Fill a scroll view with a text item
    private void AddTextComponentOnContent(Transform container, string value) {
        GameObject textGO = new GameObject();
        textGO.name = "Text-" + value;
        textGO.transform.SetParent(container);
        Text item = textGO.AddComponent<Text>();
        item.font = ArialFont;
        item.fontSize = 18;
        item.color = Color.black;

        ContentSizeFitter lCsF = textGO.AddComponent<ContentSizeFitter>();
        lCsF.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

        RectTransform rectTransform;
        rectTransform = item.GetComponent<RectTransform>();
        rectTransform.localPosition = new Vector3(0, 0, 0);
        rectTransform.sizeDelta = new Vector2(2000, 30);
        rectTransform.localScale = Vector3.one;
        item.horizontalOverflow = HorizontalWrapMode.Wrap;
        item.verticalOverflow = VerticalWrapMode.Overflow;

        item.text = value;
    }

    private void RemoveComponentsFromList(Transform container) {
        for (var i = container.childCount - 1; i >= 0; i--) {
            var obj = container.GetChild(i);
            obj.transform.SetParent(null);
            Destroy(obj.gameObject);
        }
    }

    private void UpdateUsersSession(Transform container) {
        RemoveComponentsFromList(usersSession.transform);
        if (OrchestratorController.Instance.ConnectedUsers != null) {
            foreach (User u in OrchestratorController.Instance.ConnectedUsers) {
                AddTextComponentOnContent(container.transform, u.userName);
            }
            sessionNumUsersText.text = OrchestratorController.Instance.ConnectedUsers.Length.ToString() /*+ "/" + "4"*/;
        }
        else {
            Debug.Log("[OrchestratorLogin][UpdateUsersSession] Error in Connected Users");
        }
    }

    private void UpdateSessions(Transform container, Dropdown dd) {
        RemoveComponentsFromList(container.transform);
        Array.ForEach(OrchestratorController.Instance.AvailableSessions, delegate (Session element) {
            AddTextComponentOnContent(container.transform, element.GetGuiRepresentation());
        });

        // update the dropdown
        dd.ClearOptions();
        List<Dropdown.OptionData> options = new List<Dropdown.OptionData>();
        Array.ForEach(OrchestratorController.Instance.AvailableSessions, delegate (Session sess) {
            options.Add(new Dropdown.OptionData(sess.GetGuiRepresentation()));
        });
        dd.AddOptions(options);
    }

    private void UpdateScenarios(Dropdown dd) {
        // update the dropdown
        dd.ClearOptions();
        List<Dropdown.OptionData> options = new List<Dropdown.OptionData>();
        Array.ForEach(OrchestratorController.Instance.AvailableScenarios, delegate (Scenario scenario) {
            options.Add(new Dropdown.OptionData(scenario.GetGuiRepresentation()));
        });
        dd.AddOptions(options);
    }

    private void UpdateRepresentations(Dropdown dd) {
        // Fill UserData representation dropdown according to eUserRepresentationType enum declaration
        dd.ClearOptions();
        dd.AddOptions(new List<string>(Enum.GetNames(typeof(UserData.eUserRepresentationType))));
    }

    private IEnumerator ScrollLogsToBottom() {
        yield return new WaitForSeconds(0.2f);
        logsScrollRect.verticalScrollbar.value = 0;
    }

    #endregion

    #region Unity

    // Start is called before the first frame update
    void Start() {
        if (instance == null) {
            instance = this;
        }

        system = EventSystem.current;

        // Update Application version
        orchURLText.text = orchestratorUrl;
        nativeVerText.text = VersionLog.Instance.NativeClient;
        playerVerText.text = "v" + Application.version;
        orchVerText.text = "";

        // Font to build gui components for logs!
        ArialFont = (Font)Resources.GetBuiltinResource(typeof(Font), "Arial.ttf");

        // Fill UserData representation dropdown according to eUserRepresentationType enum declaration
        UpdateRepresentations(representationTypeConfigDropdown);

        // Buttons listeners
        connectButton.onClick.AddListener(delegate { SocketConnect(); });
        okButton.onClick.AddListener(delegate { OKButton(); });
        loginButton.onClick.AddListener(delegate { Login(); });
        configButton.onClick.AddListener(delegate { ConfigButton(); });
        saveConfigButton.onClick.AddListener(delegate { SaveConfigButton(); });
        exitConfigButton.onClick.AddListener(delegate { ExitConfigButton(); });
        calibButton.onClick.AddListener(delegate { GoToCalibration(); });
        refreshSessionsButton.onClick.AddListener(delegate { GetSessions(); });
        createButton.onClick.AddListener(delegate { CreateButton(); });
        joinButton.onClick.AddListener(delegate { JoinButton(); });
        doneCreateButton.onClick.AddListener(delegate { AddSession(); });
        doneJoinButton.onClick.AddListener(delegate { JoinSession(); });
        readyButton.onClick.AddListener(delegate { ReadyButton(); });
        leaveButton.onClick.AddListener(delegate { LeaveSession(); });

        // Dropdown listeners
        representationTypeConfigDropdown.onValueChanged.AddListener(delegate { PanelChanger(); });

        InitialiseControllerEvents();

        noAudioToggle.isOn = true;
        socketAudioToggle.isOn = false;
        dashAudioToggle.isOn = false;
        presenterToggle.isOn = false;
        liveToggle.isOn = false;

        if (OrchestratorController.Instance.UserIsLogged) { // Comes from another scene
            // Set status to offline
            orchestratorConnected = true;
            statusText.text = "Online";
            statusText.color = onlineCol;
            FillSelfUserData();
            Debug.Log("Come from another Scene");
            OrchestratorController.Instance.OnLoginResponse(new ResponseStatus(), userId.text);
            //OnLogin(true);
        }
        else { // Enter for first time
            // Set status to offline
            orchestratorConnected = false;
            statusText.text = "Offline";
            statusText.color = offlineCol;
            state = State.Offline;

            // Try to connect
            SocketConnect();
        }
    }

    // Update is called once per frame
    void Update() {
        TabShortcut();
        if (state == State.Create) {
            AudioToggle();
            PresenterToggles();
        }
    }

    public void FillSelfUserData() {
        // UserID & Name
        userId.text = OrchestratorController.Instance.SelfUser.userId;
        userName.text = OrchestratorController.Instance.SelfUser.userName;
        // Config Info
        exchangeNameConfigIF.text = OrchestratorController.Instance.SelfUser.userData.userMQexchangeName;
        connectionURIConfigIF.text = OrchestratorController.Instance.SelfUser.userData.userMQurl;
        representationTypeConfigDropdown.value = (int)OrchestratorController.Instance.SelfUser.userData.userRepresentationType;
    }
    
    public void PanelChanger() {
        switch (state) {
            case State.Offline:
                // Panels
                ntpPanel.SetActive(false);
                loginPanel.SetActive(false);
                if (developerOptions)
                    usersButtonsPanel.SetActive(false);
                configPanel.SetActive(false);
                createPanel.SetActive(false);
                joinPanel.SetActive(false);
                lobbyPanel.SetActive(false);
                sessionPanel.SetActive(false);
                usersPanel.SetActive(false);
                // Buttons
                connectButton.gameObject.SetActive(true);
                break;
            case State.Online:
                // Panels
                ntpPanel.SetActive(false);
                loginPanel.SetActive(true);
                if (developerOptions)
                    usersButtonsPanel.SetActive(true);
                configPanel.SetActive(false);
                createPanel.SetActive(false);
                joinPanel.SetActive(false);
                lobbyPanel.SetActive(false);
                sessionPanel.SetActive(false);
                usersPanel.SetActive(false);
                // Buttons
                connectButton.gameObject.SetActive(false);
                configButton.gameObject.SetActive(false);
                createButton.gameObject.SetActive(false);
                joinButton.gameObject.SetActive(false);
                break;
            case State.Logged:
                // Panels
                ntpPanel.SetActive(false);
                loginPanel.SetActive(false);
                if (developerOptions)
                    usersButtonsPanel.SetActive(false);
                configPanel.SetActive(false);
                createPanel.SetActive(false);
                joinPanel.SetActive(false);
                lobbyPanel.SetActive(false);
                sessionPanel.SetActive(true);
                usersPanel.SetActive(false);
                // Buttons
                connectButton.gameObject.SetActive(false);
                configButton.gameObject.SetActive(true);
                createButton.gameObject.SetActive(true);
                joinButton.gameObject.SetActive(true);
                configButton.interactable = true;
                createButton.interactable = true;
                joinButton.interactable = true;
                break;
            case State.Config:
                // Panels
                ntpPanel.SetActive(false);
                loginPanel.SetActive(false);
                if (developerOptions)
                    usersButtonsPanel.SetActive(false);
                configPanel.SetActive(true);
                createPanel.SetActive(false);
                joinPanel.SetActive(false);
                lobbyPanel.SetActive(false);
                sessionPanel.SetActive(true);
                usersPanel.SetActive(false);
                // Buttons
                connectButton.gameObject.SetActive(false);
                configButton.gameObject.SetActive(true);
                createButton.gameObject.SetActive(true);
                joinButton.gameObject.SetActive(true);
                configButton.interactable = false;
                createButton.interactable = true;
                joinButton.interactable = true;
                // Dropdown Logic
                tvmInfoGO.SetActive(false);
                calibButton.interactable = false;
                if ((UserData.eUserRepresentationType)representationTypeConfigDropdown.value == UserData.eUserRepresentationType.__TVM__) {
                    tvmInfoGO.SetActive(true);
                    calibButton.interactable = true;
                }
                else if ((UserData.eUserRepresentationType)representationTypeConfigDropdown.value == UserData.eUserRepresentationType.__PCC_CWI_) {
                    calibButton.interactable = true;
                }
                break;
            case State.Create:
                // Panels
                ntpPanel.SetActive(false);
                loginPanel.SetActive(false);
                if (developerOptions)
                    usersButtonsPanel.SetActive(false);
                configPanel.SetActive(false);
                createPanel.SetActive(true);
                joinPanel.SetActive(false);
                lobbyPanel.SetActive(false);
                sessionPanel.SetActive(true);
                usersPanel.SetActive(false);
                // Buttons
                connectButton.gameObject.SetActive(false);
                configButton.gameObject.SetActive(true);
                createButton.gameObject.SetActive(true);
                joinButton.gameObject.SetActive(true);
                configButton.interactable = true;
                createButton.interactable = false;
                joinButton.interactable = true;
                break;
            case State.Join:
                // Panels
                ntpPanel.SetActive(false);
                loginPanel.SetActive(false);
                if (developerOptions)
                    usersButtonsPanel.SetActive(false);
                configPanel.SetActive(false);
                createPanel.SetActive(false);
                joinPanel.SetActive(true);
                lobbyPanel.SetActive(false);
                sessionPanel.SetActive(true);
                usersPanel.SetActive(false);
                // Buttons
                connectButton.gameObject.SetActive(false);
                configButton.gameObject.SetActive(true);
                createButton.gameObject.SetActive(true);
                joinButton.gameObject.SetActive(true);
                configButton.interactable = true;
                createButton.interactable = true;
                joinButton.interactable = false;
                break;
            case State.Lobby:
                // Panels
                ntpPanel.SetActive(false);
                loginPanel.SetActive(false);
                if (developerOptions)
                    usersButtonsPanel.SetActive(false);
                configPanel.SetActive(false);
                createPanel.SetActive(false);
                joinPanel.SetActive(false);
                lobbyPanel.SetActive(true);
                sessionPanel.SetActive(false);
                usersPanel.SetActive(true);
                // Buttons
                connectButton.gameObject.SetActive(false);
                configButton.gameObject.SetActive(false);
                createButton.gameObject.SetActive(true);
                joinButton.gameObject.SetActive(true);
                configButton.interactable = false;
                createButton.interactable = false;
                joinButton.interactable = false;
                if (OrchestratorController.Instance.UserIsMaster)
                    readyButton.gameObject.SetActive(true);
                else
                    readyButton.gameObject.SetActive(false);
                break;
            case State.InGame:
                break;
            default:
                break;
        }
        SelectFirstIF();
    }

    private void OnDestroy() {
        TerminateControllerEvents();
    }

    #endregion

    #region Input

    void SelectFirstIF() {
        try {
            InputField[] inputFields = FindObjectsOfType<InputField>();
            if (inputFields != null) {
                inputFields[inputFields.Length - 1].OnPointerClick(new PointerEventData(system));  //if it's an input field, also set the text caret
                inputFields[inputFields.Length - 1].caretWidth = 2;
                //system.SetSelectedGameObject(first.gameObject, new BaseEventData(system));
            }
        }
        catch { }
    }

    void TabShortcut() {
        if (Input.GetKeyDown(KeyCode.Tab)) {
            try {
                Selectable current = system.currentSelectedGameObject.GetComponent<Selectable>();
                if (current != null) {
                    Selectable next = current.FindSelectableOnDown();
                    if (next != null) {
                        InputField inputfield = next.GetComponent<InputField>();
                        if (inputfield != null) {
                            inputfield.OnPointerClick(new PointerEventData(system));  //if it's an input field, also set the text caret
                            inputfield.caretWidth = 2;
                        }

                        system.SetSelectedGameObject(next.gameObject, new BaseEventData(system));
                    }
                    else {
                        // Select the first IF because no more elements exists in the list
                        SelectFirstIF();
                    }
                }
                //else Debug.Log("no selectable object selected in event system");
            }
            catch { }
        }
    }

    #endregion

    #region Buttons

    public void OKButton() {
        PanelChanger();
    }

    public void ConfigButton() {
        state = State.Config;
        PanelChanger();
    }

    public void SaveConfigButton() {
        UpdateUserData();
        state = State.Logged;
        PanelChanger();
    }

    public void ExitConfigButton() {
        GetUserInfo();
        state = State.Logged;
        PanelChanger();
    }

    public void CreateButton() {
        state = State.Create;
        PanelChanger();
    }

    public void JoinButton() {
        state = State.Join;
        PanelChanger();
    }

    public void ReadyButton() {
        if (OrchestratorController.Instance.MyScenario.scenarioName == "Pilot 2")
            SendMessageToAll("START_" + OrchestratorController.Instance.MyScenario.scenarioName + "_" + kindAudio + "_" + kindPresenter);
        else 
            SendMessageToAll("START_" + OrchestratorController.Instance.MyScenario.scenarioName + "_" + kindAudio);
    }

    public void GoToCalibration() {
        StartCoroutine(CalibRoutine());
    }

    IEnumerator CalibRoutine() {
        UpdateUserData();
        yield return new WaitForSeconds(0.5f);
        SceneManager.LoadScene("SelfCalibration");
    }

    public void AutoFillButtons(int user) {
        switch (user) {
            case 0:
                userNameLoginIF.text = "Marc@i2CAT";
                userPasswordLoginIF.text = "i2CAT2020";
                break;
            case 1:
                userNameLoginIF.text = "Luca@i2CAT";
                userPasswordLoginIF.text = "i2CAT2020";
                break;
            case 2:
                userNameLoginIF.text = "Einar@i2CAT";
                userPasswordLoginIF.text = "i2CAT2020";
                break;
            case 3:
                userNameLoginIF.text = "Isaac@i2CAT";
                userPasswordLoginIF.text = "i2CAT2020";
                break;
            case 4:
                userNameLoginIF.text = "cwibig";
                userPasswordLoginIF.text = "CWI2020";
                break;
            case 5:
                userNameLoginIF.text = "cwismall";
                userPasswordLoginIF.text = "CWI2020";
                break;
            case 6:
                userNameLoginIF.text = "cwitiny";
                userPasswordLoginIF.text = "CWI2020";
                break;
            case 7:
                userNameLoginIF.text = "Jack@CWI";
                userPasswordLoginIF.text = "CWI2020";
                break;
            case 8:
                userNameLoginIF.text = "Shishir@CWI";
                userPasswordLoginIF.text = "CWI2020";
                break;
            case 9:
                userNameLoginIF.text = "Fernando@THEMO";
                userPasswordLoginIF.text = "THEMO2020";
                break;
            case 10:
                userNameLoginIF.text = "Romain@MS";
                userPasswordLoginIF.text = "MS2020";
                break;
            case 11:
                userNameLoginIF.text = "Argyris@CERTH";
                userPasswordLoginIF.text = "CERTH2020";
                break;
            case 12:
                userNameLoginIF.text = "Spiros@CERTH";
                userPasswordLoginIF.text = "CERTH2020";
                break;
            case 13:
                userNameLoginIF.text = "Vincent@VO";
                userPasswordLoginIF.text = "VO2020";
                break;
            case 14:
                userNameLoginIF.text = "Patrice@VO";
                userPasswordLoginIF.text = "VO2020";
                break;
            case 15:
                userNameLoginIF.text = "Name";
                userPasswordLoginIF.text = "Lastname";
                break;
            default:
                break;
        }
    }

    #endregion

    #region Toggles 

    private void AudioToggle() {
        if (noAudioToggle.isOn)
            noAudioToggle.interactable = false;
        else
            noAudioToggle.interactable = true;
        if (socketAudioToggle.isOn) 
            socketAudioToggle.interactable = false;
        else 
            socketAudioToggle.interactable = true;
        if (dashAudioToggle.isOn)
            dashAudioToggle.interactable = false;
        else
            dashAudioToggle.interactable = true;
    }

    public void SetAudio(int kind) {
        switch (kind) {
            case 0: // No
                if (noAudioToggle.isOn) {
                    // Set AudioType
                    Config.Instance.audioType = Config.AudioType.None;
                    // Set Toggles
                    socketAudioToggle.isOn = false;
                    dashAudioToggle.isOn = false;
                }
                break;
            case 1: // Socket
                if (socketAudioToggle.isOn) {
                    // Set AudioType
                    Config.Instance.audioType = Config.AudioType.SocketIO;
                    // Set Toggles
                    noAudioToggle.isOn = false;
                    dashAudioToggle.isOn = false;
                }
                break;
            case 2: // Dash
                if (dashAudioToggle.isOn) {
                    // Set AudioType
                    Config.Instance.audioType = Config.AudioType.Dash;
                    // Set Toggles
                    noAudioToggle.isOn = false;
                    socketAudioToggle.isOn = false;
                }
                break;
            default:
                break;
        }
        kindAudio = kind;
    }

    private void PresenterToggles() {
        if (OrchestratorController.Instance.AvailableScenarios[scenarioIdDrop.value].scenarioName == "Pilot 2") {
            presenterToggle.gameObject.SetActive(true);
            // Check if presenter is active to show live option
            if (presenterToggle.isOn) {
                liveToggle.gameObject.SetActive(true);
                if (liveToggle.isOn) {
                    Config.Instance.presenter = Config.Presenter.Live;
                    kindPresenter = 2;
                }
                else {
                    Config.Instance.presenter = Config.Presenter.Local;
                    kindPresenter = 1;
                }
            }
            else {
                liveToggle.gameObject.SetActive(false);
                Config.Instance.presenter = Config.Presenter.None;
                kindPresenter = 0;
            }  
        }
        else {
            presenterToggle.gameObject.SetActive(false);
            liveToggle.gameObject.SetActive(false);
        }
    }      

    #endregion

    #region Events listeners

    // Subscribe to Orchestrator Wrapper Events
    private void InitialiseControllerEvents() {
        OrchestratorController.Instance.OnConnectionEvent += OnConnect;
        OrchestratorController.Instance.OnConnectionEvent += OnDisconnect;
        OrchestratorController.Instance.OnOrchestratorRequestEvent += OnOrchestratorRequest;
        OrchestratorController.Instance.OnOrchestratorResponseEvent += OnOrchestratorResponse;
        OrchestratorController.Instance.OnGetOrchestratorVersionEvent += OnGetOrchestratorVersionHandler;
        OrchestratorController.Instance.OnLoginEvent += OnLogin;
        OrchestratorController.Instance.OnLogoutEvent += OnLogout;
        OrchestratorController.Instance.OnGetNTPTimeEvent += OnGetNTPTimeResponse;
        OrchestratorController.Instance.OnGetSessionsEvent += OnGetSessionsHandler;
        OrchestratorController.Instance.OnAddSessionEvent += OnAddSessionHandler;
        OrchestratorController.Instance.OnJoinSessionEvent += OnJoinSessionHandler;
        OrchestratorController.Instance.OnLeaveSessionEvent += OnLeaveSessionHandler;
        OrchestratorController.Instance.OnDeleteSessionEvent += OnDeleteSessionHandler;
        OrchestratorController.Instance.OnUserJoinSessionEvent += OnUserJoinedSessionHandler;
        OrchestratorController.Instance.OnUserLeaveSessionEvent += OnUserLeftSessionHandler;
        OrchestratorController.Instance.OnGetScenarioEvent += OnGetScenarioInstanceInfoHandler;
        OrchestratorController.Instance.OnGetScenariosEvent += OnGetScenariosHandler;
        OrchestratorController.Instance.OnGetLiveDataEvent += OnGetLivePresenterDataHandler;
        OrchestratorController.Instance.OnGetUsersEvent += OnGetUsersHandler;
        OrchestratorController.Instance.OnAddUserEvent += OnAddUserHandler;
        OrchestratorController.Instance.OnGetUserInfoEvent += OnGetUserInfoHandler;
        OrchestratorController.Instance.OnGetRoomsEvent += OnGetRoomsHandler;
        OrchestratorController.Instance.OnJoinRoomEvent += OnJoinRoomHandler;
        OrchestratorController.Instance.OnLeaveRoomEvent += OnLeaveRoomHandler;
        OrchestratorController.Instance.OnUserMessageReceivedEvent += OnUserMessageReceivedHandler;
        OrchestratorController.Instance.OnMasterEventReceivedEvent += OnMasterEventReceivedHandler;
        OrchestratorController.Instance.OnUserEventReceivedEvent += OnUserEventReceivedHandler;
    }

    // Un-Subscribe to Orchestrator Wrapper Events
    private void TerminateControllerEvents() {
        OrchestratorController.Instance.OnConnectionEvent -= OnConnect;
        OrchestratorController.Instance.OnConnectionEvent -= OnDisconnect;
        OrchestratorController.Instance.OnOrchestratorRequestEvent -= OnOrchestratorRequest;
        OrchestratorController.Instance.OnOrchestratorResponseEvent -= OnOrchestratorResponse;
        OrchestratorController.Instance.OnGetOrchestratorVersionEvent -= OnGetOrchestratorVersionHandler;
        OrchestratorController.Instance.OnLoginEvent -= OnLogin;
        OrchestratorController.Instance.OnLogoutEvent -= OnLogout;
        OrchestratorController.Instance.OnGetNTPTimeEvent -= OnGetNTPTimeResponse;
        OrchestratorController.Instance.OnGetSessionsEvent -= OnGetSessionsHandler;
        OrchestratorController.Instance.OnAddSessionEvent -= OnAddSessionHandler;
        OrchestratorController.Instance.OnJoinSessionEvent -= OnJoinSessionHandler;
        OrchestratorController.Instance.OnLeaveSessionEvent -= OnLeaveSessionHandler;
        OrchestratorController.Instance.OnDeleteSessionEvent -= OnDeleteSessionHandler;
        OrchestratorController.Instance.OnUserJoinSessionEvent -= OnUserJoinedSessionHandler;
        OrchestratorController.Instance.OnUserLeaveSessionEvent -= OnUserLeftSessionHandler;
        OrchestratorController.Instance.OnGetScenarioEvent -= OnGetScenarioInstanceInfoHandler;
        OrchestratorController.Instance.OnGetScenariosEvent -= OnGetScenariosHandler;
        OrchestratorController.Instance.OnGetLiveDataEvent -= OnGetLivePresenterDataHandler;
        OrchestratorController.Instance.OnGetUsersEvent -= OnGetUsersHandler;
        OrchestratorController.Instance.OnAddUserEvent -= OnAddUserHandler;
        OrchestratorController.Instance.OnGetUserInfoEvent -= OnGetUserInfoHandler;
        OrchestratorController.Instance.OnGetRoomsEvent -= OnGetRoomsHandler;
        OrchestratorController.Instance.OnJoinRoomEvent -= OnJoinRoomHandler;
        OrchestratorController.Instance.OnLeaveRoomEvent -= OnLeaveRoomHandler;
        OrchestratorController.Instance.OnUserMessageReceivedEvent -= OnUserMessageReceivedHandler;
        OrchestratorController.Instance.OnMasterEventReceivedEvent -= OnMasterEventReceivedHandler;
        OrchestratorController.Instance.OnUserEventReceivedEvent -= OnUserEventReceivedHandler;
    }

    #endregion

    #region Commands

    #region Socket.io connect

    public void SocketConnect() {
        OrchestratorController.Instance.SocketConnect(orchestratorUrl);
    }

    private void OnConnect(bool pConnected) {
        if (pConnected) {
            orchestratorConnected = true;
            statusText.text = "Online";
            statusText.color = onlineCol;
            state = State.Online;
        }
        PanelChanger();
    }

    private void socketDisconnect() {
        OrchestratorController.Instance.socketDisconnect();
    }

    private void OnDisconnect(bool pConnected) {
        if (!pConnected) {
            OnLogout(true);
            orchestratorConnected = false;
            statusText.text = "Offline";
            statusText.color = offlineCol;
            state = State.Offline;
        }
        PanelChanger();
    }

    private void OnGetOrchestratorVersionHandler(string pVersion) {
        Debug.Log("Orchestration Service: " + pVersion);
        orchVerText.text = pVersion;
        OrchestratorController.Instance.GetNTPTime();
    }

    #endregion

    #region Orchestrator Logs

    // Display the sent message in the logs
    public void OnOrchestratorRequest(string pRequest) {
        AddTextComponentOnContent(logsContainer.transform, ">>> " + pRequest);
    }

    // Display the received message in the logs
    public void OnOrchestratorResponse(string pResponse) {
        AddTextComponentOnContent(logsContainer.transform, "<<< " + pResponse);
        StartCoroutine(ScrollLogsToBottom());
    }

    #endregion

    #region Login/Logout

    // Login from the main buttons Login & Logout
    private void Login() {
        OrchestratorController.Instance.Login(userNameLoginIF.text, userPasswordLoginIF.text);
    }

    private void OnLogin(bool userLoggedSucessfully) {
        if (userLoggedSucessfully) {
            OrchestratorController.Instance.IsAutoRetrievingData = autoRetrieveOrchestratorDataOnConnect;

            // UserData info in Login
            //UserData lUserData = new UserData {
            //    userMQexchangeName = exchangeNameLoginIF.text,
            //    userMQurl = connectionURILoginIF.text,
            //    userRepresentationType = (UserData.eUserRepresentationType)representationTypeLoginDropdown.value
            //};
            //OrchestratorController.Instance.UpdateUserData(lUserData);

            state = State.Logged;
        }
        else {
            this.userId.text = "";
            userName.text = "";
        }

        userLogged = userLoggedSucessfully;
        PanelChanger();
    }
    
    private void Logout() {
        OrchestratorController.Instance.Logout();
    }

    private void OnLogout(bool userLogoutSucessfully) {
        if (userLogoutSucessfully) {
            userLogged = false;
            userId.text = "";
            userName.text = "";
            state = State.Online;
        }
        PanelChanger();
    }

    #endregion

    #region NTP clock

    private void GetNTPTime() {
        OrchestratorController.Instance.GetNTPTime();
    }

    private void OnGetNTPTimeResponse(NtpClock ntpTime) {
        int difference = Helper.GetClockTimestamp(DateTime.UtcNow) - ntpTime.Timestamp;
        if (difference >= ntpSyncThreshold || difference <= -ntpSyncThreshold) {
            ntpText.text = "You have a desynchronization of " + difference + " sec with the Orchestrator.\nYou may suffer some problems as a result.";
            ntpPanel.SetActive(true);
            loginPanel.SetActive(false);
        }
        Debug.Log("[OrchestratorLogin][OnGetNTPTimeResponse] Difference: " + difference);
    }

    #endregion

    #region Sessions

    private void GetSessions() {
        OrchestratorController.Instance.GetSessions();
    }

    private void OnGetSessionsHandler(Session[] sessions) {
        if (sessions != null) {
            // update the list of available sessions
            UpdateSessions(orchestratorSessions, sessionIdDrop);
        }
    }

    private void AddSession() {
        OrchestratorController.Instance.AddSession(OrchestratorController.Instance.AvailableScenarios[scenarioIdDrop.value].scenarioId,
                                                    sessionNameIF.GetComponentInChildren<InputField>().text,
                                                    sessionDescriptionIF.GetComponentInChildren<InputField>().text);
    }

    private void OnAddSessionHandler(Session session) {
        // Is equal to AddSession + Join Session, except that session is returned (not on JoinSession)
        if (session != null) {
            // update the list of available sessions
            UpdateSessions(orchestratorSessions, sessionIdDrop);

            // Update the info in LobbyPanel
            isMaster = OrchestratorController.Instance.UserIsMaster;
            sessionNameText.text = session.sessionName;
            sessionDescriptionText.text = session.sessionDescription;

            // Update the list of session users
            UpdateUsersSession(usersSession);

            state = State.Lobby;
            PanelChanger();
        }
        else {
            isMaster = false;
            sessionNameText.text = "";
            sessionDescriptionText.text = "";
            scenarioIdText.text = "";
            sessionNumUsersText.text = "";
            RemoveComponentsFromList(usersSession.transform);
        }
    }

    private void OnGetScenarioInstanceInfoHandler(ScenarioInstance scenario) {
        if (scenario != null) {
            scenarioIdText.text = scenario.scenarioName;
            // Update the list of session users
            UpdateUsersSession(usersSession);
        }
    }

    private void DeleteSession() {
        OrchestratorController.Instance.DeleteSession(OrchestratorController.Instance.MySession.sessionId);
    }

    private void OnDeleteSessionHandler() {
        Debug.Log("[OrchestratorLogin][OnDeleteSessionHandler] Not implemented");
    }

    private void JoinSession() {
        string sessionIdToJoin = OrchestratorController.Instance.AvailableSessions[sessionIdDrop.value].sessionId;
        OrchestratorController.Instance.JoinSession(sessionIdToJoin);
        joining = true;
    }

    private void OnJoinSessionHandler(Session session) {
        if (session != null) {
            isMaster = OrchestratorController.Instance.UserIsMaster;
            // Update the info in LobbyPanel
            sessionNameText.text = session.sessionName;
            sessionDescriptionText.text = session.sessionDescription;

            // Update the list of session users
            UpdateUsersSession(usersSession);

            state = State.Lobby;
            PanelChanger();
        }
        else {
            isMaster = false;
            sessionNameText.text = "";
            sessionDescriptionText.text = "";
            scenarioIdText.text = "";
            sessionNumUsersText.text = "";
            RemoveComponentsFromList(usersSession.transform);
        }
    }

    private void LeaveSession() {
        OrchestratorController.Instance.LeaveSession();
        joining = false;
    }

    private void OnLeaveSessionHandler() {
        isMaster = false;
        sessionNameText.text = "";
        sessionDescriptionText.text = "";
        scenarioIdText.text = "";
        sessionNumUsersText.text = "";
        RemoveComponentsFromList(usersSession.transform);

        state = State.Logged;
        PanelChanger();
    }

    private void OnUserJoinedSessionHandler(string userID) {
        if (!string.IsNullOrEmpty(userID)) {
            UpdateUsersSession(usersSession);
            if (!joining)
                OrchestratorController.Instance.GetUserInfo(userID);
        }
    }

    private void OnUserLeftSessionHandler(string userID) {
        if (!string.IsNullOrEmpty(userID)) {
            UpdateUsersSession(usersSession);
            OrchestratorController.Instance.GetUsers();
        }
    }

    #endregion

    #region Scenarios

    private void GetScenarios() {
        OrchestratorController.Instance.GetScenarios();
    }

    private void OnGetScenariosHandler(Scenario[] scenarios) {
        if (scenarios != null && scenarios.Length > 0) {
            //update the data in the dropdown
            UpdateScenarios(scenarioIdDrop);
        }
    }

    #endregion

    #region Live

    private void OnGetLivePresenterDataHandler(LivePresenterData liveData) {
        //Debug.Log("[OrchestratorLogin][OnGetLivePresenterDataHandler] Not implemented");
    }

    #endregion

    #region Users

    private void GetUsers() {
        OrchestratorController.Instance.GetUsers();
    }

    private void OnGetUsersHandler(User[] users) {
        Debug.Log("[OrchestratorLogin][OnGetUsersHandler] Not implemented");

        UpdateUsersSession(usersSession);

        // Update the sfuData if is in session.
        if (OrchestratorController.Instance.ConnectedUsers != null) {
            for (int i = 0; i < OrchestratorController.Instance.ConnectedUsers.Length; ++i) {
                foreach (User u in users) {
                    if (OrchestratorController.Instance.ConnectedUsers[i].userId == u.userId) {
                        OrchestratorController.Instance.ConnectedUsers[i].sfuData = u.sfuData;
                    }
                }
            }
        }
    }

    private void AddUser() {
        Debug.Log("[OrchestratorLogin][AddUser] Not implemented");
    }

    private void OnAddUserHandler(User user) {
        Debug.Log("[OrchestratorLogin][OnAddUserHandler] Not implemented");
    }

    private void UpdateUserData() {
        // UserData info in Config
        UserData lUserData = new UserData {
            userMQexchangeName = exchangeNameConfigIF.text,
            userMQurl = connectionURIConfigIF.text,
            userRepresentationType = (UserData.eUserRepresentationType)representationTypeConfigDropdown.value
        };
        OrchestratorController.Instance.UpdateUserData(lUserData);
    }

    private void GetUserInfo() {
        OrchestratorController.Instance.GetUserInfo(OrchestratorController.Instance.SelfUser.userId);
    }

    private void OnGetUserInfoHandler(User user) {
        if (user != null) {
            if (string.IsNullOrEmpty(userId.text) || user.userId == OrchestratorController.Instance.SelfUser.userId) {
                OrchestratorController.Instance.SelfUser = user;

                userId.text = user.userId;
                userName.text = user.userName;

                //UserData
                exchangeNameConfigIF.text = user.userData.userMQexchangeName;
                connectionURIConfigIF.text = user.userData.userMQurl;
                representationTypeConfigDropdown.value = (int)user.userData.userRepresentationType;
            }

            UpdateUsersSession(usersSession);

            // Update the sfuData and UserData if is in session.
            if (OrchestratorController.Instance.ConnectedUsers != null) {
                for (int i = 0; i < OrchestratorController.Instance.ConnectedUsers.Length; ++i) {
                    if (OrchestratorController.Instance.ConnectedUsers[i].userId == user.userId) {
                        // sfuData
                        OrchestratorController.Instance.ConnectedUsers[i].sfuData = user.sfuData;
                        // UserData
                        OrchestratorController.Instance.ConnectedUsers[i].userData = user.userData;
                    }
                }
            }
        }
    }

    private void DeleteUser() {
        Debug.Log("[OrchestratorLogin][DeleteUser] Not implemented");
    }

    #endregion

    #region Rooms

    private void GetRooms() {
        OrchestratorController.Instance.GetRooms();
    }

    private void OnGetRoomsHandler(RoomInstance[] rooms) {
        Debug.Log("[OrchestratorLogin][OnGetRoomsHandler] Not implemented");
    }

    private void JoinRoom() {
        Debug.Log("[OrchestratorLogin][JoinRoom] Not implemented");
    }

    private void OnJoinRoomHandler(bool hasJoined) {
        Debug.Log("[OrchestratorLogin][OnJoinRoomHandler] Not implemented");
    }

    private void LeaveRoom() {
        OrchestratorController.Instance.LeaveRoom();
    }

    private void OnLeaveRoomHandler() {
        Debug.Log("[OrchestratorLogin][OnLeaveRoomHandler] Not implemented");
    }

    #endregion

    #region Messages

    private void SendMessage() {
        Debug.Log("[OrchestratorLogin][SendMessage] Not implemented");
    }

    private void SendMessageToAll(string message) {
        OrchestratorController.Instance.SendMessageToAll(message);
    }

    private void OnUserMessageReceivedHandler(UserMessage userMessage) {
        AddTextComponentOnContent(logsContainer.transform, "<<< USER MESSAGE RECEIVED: " + userMessage.fromName + "[" + userMessage.fromId + "]: " + userMessage.message);
        StartCoroutine(ScrollLogsToBottom());

        LoginController.Instance.MessageActivation(userMessage.message);
    }

    #endregion

    #region Events

    private void SendEventToMaster() {
        Debug.Log("[OrchestratorLogin][SendEventToMaster] Not implemented");
    }

    private void SendEventToUser() {
        Debug.Log("[OrchestratorLogin][SendEventToUser] Not implemented");
    }

    private void SendEventToAll() {
        Debug.Log("[OrchestratorLogin][SendEventToAll] Not implemented");
    }

    private void OnMasterEventReceivedHandler(UserEvent pMasterEventData) {
        Debug.Log("[OrchestratorLogin][OnMasterEventReceivedHandler] Not implemented");
    }

    private void OnUserEventReceivedHandler(UserEvent pUserEventData) {
        Debug.Log("[OrchestratorLogin][OnUserEventReceivedHandler] Not implemented");
    }

    #endregion

    #region Data Stream

    private void GetAvailableDataStreams() {
        Debug.Log("[OrchestratorLogin][GetAvailableDataStreams] Not implemented");
    }

    private void GetRegisteredDataStreams() {
        OrchestratorController.Instance.GetRegisteredDataStreams();
    }

    #endregion

    #endregion

#if UNITY_STANDALONE_WIN
    void OnGUI() {
        if (GUI.Button(new Rect(Screen.width / 2, 5, 70, 20), "Open Log")) {
            var log_path = System.IO.Path.Combine(System.IO.Directory.GetParent(Environment.GetEnvironmentVariable("AppData")).ToString(), "LocalLow", Application.companyName, Application.productName, "Player.log");
            Debug.Log(log_path);
            Application.OpenURL(log_path);
        }
    }
#endif
}
