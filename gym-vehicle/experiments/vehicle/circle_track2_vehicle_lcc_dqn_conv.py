#!/usr/bin/env python
"""
Based on:
https://github.com/vmayoral/basic_reinforcement_learning
https://gist.github.com/wingedsheep/4199594b02138dd427c22a540d6d6b8d
"""
import gym
import gym_vehicle
import time
from distutils.dir_util import copy_tree
import os
import json
import random
import numpy as np
from keras.models import Sequential, load_model
from keras import optimizers
from keras.layers import Conv1D, MaxPooling1D, Flatten
from keras.layers.core import Dense, Dropout, Activation
from keras.layers.normalization import BatchNormalization
from keras.layers.advanced_activations import LeakyReLU
from keras.regularizers import l2
import memory
import matplotlib
import matplotlib.pyplot as plt
import itertools


class LivePlot(object):
    def __init__(self, outdir, data_key='episode_rewards', line_color='blue'):
        """
        Liveplot renders a graph of either episode_rewards or episode_lengths
        Args:
            outdir (outdir): Monitor output file location used to populate the graph
            data_key (Optional[str]): The key in the json to graph (episode_rewards or episode_lengths).
            line_color (Optional[dict]): Color of the plot.
        """
        self.outdir = outdir
        self._last_data = None
        self.data_key = data_key
        self.line_color = line_color

        #styling options
        matplotlib.rcParams['toolbar'] = 'None'
        plt.style.use('ggplot')
        plt.xlabel("")
        plt.ylabel(data_key)
        fig = plt.gcf().canvas.set_window_title('simulation_graph')

    def expand(self, lst, n):
        lst = [[i]*n for i in lst]
        lst = list(itertools.chain.from_iterable(lst))
        return lst

    def plot(self, reward):
        # results = monitoring.load_results(self.outdir)
        # print(results)
        # if(results==None): return

        data = reward
        #only update plot if data is different (plot calls are expensive)
        # if data !=  self._last_data:
        self._last_data = data
        plt.plot(data, color=self.line_color)

        avg_data = []
        average = 10
        for i, val in enumerate(data):
            if i%average==0:
                if (i+average) < len(data):
                    avg =  sum(data[i:i+average])/average
                    avg_data.append(avg)
        new_data = self.expand(avg_data,average)
        plt.plot(new_data, color='red', linewidth=2.5) 


        # pause so matplotlib will display
        # may want to figure out matplotlib animation or use a different library in the future
        plt.pause(0.05)

    def save(self, outdir, epoch):
        # results = monitoring.load_results(self.outdir)
        # print(results)
        # if(results==None): return

        plt.savefig(outdir+'vehicle-reward_history-epoch-'+str(epoch), format='png')

        # pause so matplotlib will display
        # may want to figure out matplotlib animation or use a different library in the future
        plt.pause(0.05)


class DeepQ:
    """
    DQN abstraction.

    As a quick reminder:
        traditional Q-learning:
            Q(s, a) += alpha * (reward(s,a) + gamma * max(Q(s') - Q(s,a))
        DQN:
            target = reward(s,a) + gamma * max(Q(s')

    """
    def __init__(self, inputs, outputs, memorySize, discountFactor, learningRate, learnStart):
        """
        Parameters:
            - inputs: input size
            - outputs: output size
            - memorySize: size of the memory that will store each state
            - discountFactor: the discount factor (gamma)
            - learningRate: learning rate
            - learnStart: steps to happen before for learning. Set to 128
        """
        self.input_size = inputs
        self.output_size = outputs
        self.memory = memory.Memory(memorySize)
        self.discountFactor = discountFactor
        self.learnStart = learnStart
        self.learningRate = learningRate

    def initNetworks(self, hiddenLayers):
        model = self.createModel(self.input_size, self.output_size, hiddenLayers, "relu", self.learningRate)
        self.model = model

        targetModel = self.createModel(self.input_size, self.output_size, hiddenLayers, "relu", self.learningRate)
        self.targetModel = targetModel

    def createRegularizedModel(self, inputs, outputs, hiddenLayers, activationType, learningRate):
        bias = True
        dropout = 0
        regularizationFactor = 0.01
        model = Sequential()
        if len(hiddenLayers) == 0:
            model.add(Dense(self.output_size, input_shape=(self.input_size,), kernel_initializer='lecun_uniform', bias=bias))
            model.add(Activation("linear"))
        else:
            if regularizationFactor > 0:
                model.add(Dense(hiddenLayers[0], input_shape=(self.input_size,), kernel_initializer='lecun_uniform', W_regularizer=l2(regularizationFactor), bias=bias))
            else:
                model.add(Dense(hiddenLayers[0], input_shape=(self.input_size,), kernel_initializer='lecun_uniform', bias=bias))

            if activationType == "LeakyReLU":
                model.add(LeakyReLU(alpha=0.01))
            else:
                model.add(Activation(activationType))

            for index in range(1, len(hiddenLayers)):
                layerSize = hiddenLayers[index]
                if regularizationFactor > 0:
                    model.add(Dense(layerSize, kernel_initializer='lecun_uniform', W_regularizer=l2(regularizationFactor), bias=bias))
                else:
                    model.add(Dense(layerSize, kernel_initializer='lecun_uniform', bias=bias))
                if activationType == "LeakyReLU":
                    model.add(LeakyReLU(alpha=0.01))
                else:
                    model.add(Activation(activationType))
                if dropout > 0:
                    model.add(Dropout(dropout))
            model.add(Dense(self.output_size, kernel_initializer='lecun_uniform', bias=bias))
            model.add(Activation("linear"))
        optimizer = optimizers.RMSprop(lr=learningRate, rho=0.9, epsilon=1e-06)
        model.compile(loss="mse", optimizer=optimizer)
        model.summary()
        return model

    def createModel(self, inputs, outputs, hiddenLayers, activationType, learningRate):
        model = Sequential()
        model.add(Conv1D(32, 2, input_shape=(self.input_size, 1), kernel_initializer='lecun_uniform'))
        model.add(Activation("relu"))
        model.add(Conv1D(32, 2, kernel_initializer='lecun_uniform'))
        model.add(Activation("relu"))
        model.add(BatchNormalization())
        model.add(MaxPooling1D(pool_size=2))
        model.add(Dropout(0.4))
        model.add(Flatten())
        model.add(Dense(32, kernel_initializer='lecun_uniform'))
        model.add(Activation("relu"))
        model.add(Dense(64, kernel_initializer='lecun_uniform'))
        model.add(Activation("relu"))
        model.add(Dense(self.output_size, kernel_initializer='lecun_uniform'))
        model.add(Activation("linear"))
        optimizer = optimizers.RMSprop(lr=learningRate, rho=0.9, epsilon=1e-06)
        model.compile(loss="mse", optimizer=optimizer)
        model.summary()
        return model

    def printNetwork(self):
        i = 0
        for layer in self.model.layers:
            weights = layer.get_weights()
            print("layer ", i, ": ", weights)
            i += 1

    def backupNetwork(self, model, backup):
        weightMatrix = []
        for layer in model.layers:
            weights = layer.get_weights()
            weightMatrix.append(weights)
        i = 0
        for layer in backup.layers:
            weights = weightMatrix[i]
            layer.set_weights(weights)
            i += 1

    def updateTargetNetwork(self):
        self.backupNetwork(self.model, self.targetModel)

    # predict Q values for all the actions
    def getQValues(self, state):
        predicted = self.model.predict(state.reshape(1, len(state), 1))
        return predicted[0]

    def getTargetQValues(self, state):
        predicted = self.targetModel.predict(state.reshape(1, len(state), 1))
        return predicted[0]

    def getMaxQ(self, qValues):
        return np.max(qValues)

    def getMaxIndex(self, qValues):
        return np.argmax(qValues)

    # calculate the target function
    def calculateTarget(self, qValuesNewState, reward, isFinal):
        """
        target = reward(s,a) + gamma * max(Q(s')
        """
        if isFinal:
            return reward
        else:
            return reward + self.discountFactor * self.getMaxQ(qValuesNewState)

    # select the action with the highest Q value
    def selectAction(self, qValues, explorationRate):
        rand = random.random()
        if rand < explorationRate:
            action = np.random.randint(0, self.output_size)
        else:
            action = self.getMaxIndex(qValues)
        return action

    def selectActionByProbability(self, qValues, bias):
        qValueSum = 0
        shiftBy = 0
        for value in qValues:
            if value + shiftBy < 0:
                shiftBy = - (value + shiftBy)
        shiftBy += 1e-06

        for value in qValues:
            qValueSum += (value + shiftBy) ** bias

        probabilitySum = 0
        qValueProbabilities = []
        for value in qValues:
            probability = ((value + shiftBy) ** bias) / float(qValueSum)
            qValueProbabilities.append(probability + probabilitySum)
            probabilitySum += probability
        qValueProbabilities[len(qValueProbabilities) - 1] = 1

        rand = random.random()
        i = 0
        for value in qValueProbabilities:
            if rand <= value:
                return i
            i += 1

    def addMemory(self, state, action, reward, newState, isFinal):
        self.memory.addMemory(state, action, reward, newState, isFinal)

    def learnOnLastState(self):
        if self.memory.getCurrentSize() >= 1:
            return self.memory.getMemory(self.memory.getCurrentSize() - 1)

    def learnOnMiniBatch(self, miniBatchSize, useTargetNetwork=True):
        # Do not learn until we've got self.learnStart samples
        if self.memory.getCurrentSize() > self.learnStart:
            # learn in batches of 128
            miniBatch = self.memory.getMiniBatch(miniBatchSize)
            X_batch = np.empty((0, self.input_size), dtype=np.float64)
            Y_batch = np.empty((0, self.output_size), dtype=np.float64)
            for sample in miniBatch:
                isFinal = sample['isFinal']
                state = sample['state']
                action = sample['action']
                reward = sample['reward']
                newState = sample['newState']

                qValues = self.getQValues(state)
                if useTargetNetwork:
                    qValuesNewState = self.getTargetQValues(newState)
                else :
                    qValuesNewState = self.getQValues(newState)
                targetValue = self.calculateTarget(qValuesNewState, reward, isFinal)

                X_batch = np.append(X_batch, np.array([state.copy()]), axis=0)
                Y_sample = qValues.copy()
                Y_sample[action] = targetValue
                Y_batch = np.append(Y_batch, np.array([Y_sample]), axis=0)
                if isFinal:
                    X_batch = np.append(X_batch, np.array([newState.copy()]), axis=0)
                    Y_batch = np.append(Y_batch, np.array([[reward]*self.output_size]), axis=0)
            self.model.fit(X_batch.reshape(X_batch.shape[0], X_batch.shape[1], 1), Y_batch, batch_size=len(miniBatch), epochs=1, verbose=0)

    def saveModel(self, path):
        self.model.save(path)

    def loadWeights(self, path):
        self.model.set_weights(load_model(path).get_weights())


def detect_monitor_files(training_dir):
    return [os.path.join(training_dir, f) for f in os.listdir(training_dir) if f.startswith('openaigym')]


def clear_monitor_files(training_dir):
    files = detect_monitor_files(training_dir)
    if len(files) == 0:
        return
    for file in files:
        print(file)
        os.unlink(file)


if __name__ == '__main__':

    env = gym.make('GazeboCircletrack2VehicleLcc-v0')
    
    outdir = '../../results/circle2_lcc_conv/'
    if not os.path.exists(outdir):
        os.mkdir(outdir, 0755)
    
    plotter = LivePlot(outdir)

    continue_execution = False
    #fill this if continue_execution=True

    model_output = '../../saved_models/circle2_lcc_conv/'
    weights_path = model_output + 'circle2_conv_ep1000.h5'
    monitor_path = model_output + 'circle2_conv_ep1000'
    params_json  = model_output + 'circle2_conv_ep1000.json'
    if not os.path.exists(model_output):
        os.mkdir(model_output, 0755)
        
    if not continue_execution:
        #Each time we take a sample and update our weights it is called a mini-batch.
        #Each time we run through the entire dataset, it's called an epoch.
        #PARAMETER LIST
        epochs = 5000
        steps = 2000
        updateTargetNetwork = 10000
        explorationRate = 1
        minibatch_size = 64
        learnStart = 64
        learningRate = 0.00025
        discountFactor = 0.99
        memorySize = 1000000
        network_inputs = 16
        network_outputs = 3
        network_structure = [300, 300]
        current_epoch = 0

        deepQ = DeepQ(network_inputs, network_outputs, memorySize, discountFactor, learningRate, learnStart)
        deepQ.initNetworks(network_structure)
        # env.monitor.start(outdir, force=True, seed=None)
        gym.wrappers.Monitor(env, outdir, force=True)
    else:
        #Load weights, monitor info and parameter info.
        #ADD TRY CATCH fro this else
        with open(params_json) as outfile:
            d = json.load(outfile)
            epochs = d.get('epochs') * 10
            steps = d.get('steps')
            updateTargetNetwork = d.get('updateTargetNetwork')
            explorationRate = d.get('explorationRate')
            minibatch_size = d.get('minibatch_size')
            learnStart = d.get('learnStart')
            learningRate = d.get('learningRate')
            discountFactor = d.get('discountFactor')
            memorySize = d.get('memorySize')
            network_inputs = d.get('network_inputs')
            network_outputs = d.get('network_outputs')
            network_structure = d.get('network_structure')
            current_epoch = d.get('current_epoch')

        deepQ = DeepQ(network_inputs, network_outputs, memorySize, discountFactor, learningRate, learnStart)
        deepQ.initNetworks(network_structure)
        deepQ.loadWeights(weights_path)

        clear_monitor_files(outdir)
        # copy_tree(monitor_path, outdir)
        # env.monitor.start(outdir, force=True, seed=None)
        gym.wrappers.Monitor(env, outdir, force=True)

    last100Scores = [0] * 100
    last100ScoresIndex = 0
    last100Filled = False
    stepCounter = 0
    highest_reward = 0

    start_time = time.time()

    list_rewards = []

    #start iterating from 'current epoch'.

    for epoch in xrange(current_epoch+1, epochs+1, 1):
        observation = env.reset()
        cumulated_reward = 0

        # number of timesteps
        for t in xrange(steps):
            # env.render()
            qValues = deepQ.getQValues(observation)

            action = deepQ.selectAction(qValues, explorationRate)

            newObservation, reward, done, info = env.step(action)

            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            deepQ.addMemory(observation, action, reward, newObservation, done)

            if stepCounter >= learnStart:
                if stepCounter <= updateTargetNetwork:
                    deepQ.learnOnMiniBatch(minibatch_size, False)
                else:
                    deepQ.learnOnMiniBatch(minibatch_size, True)

            observation = newObservation

            # env.monitor.flush(force=True)
            if done:
                last100Scores[last100ScoresIndex] = t
                last100ScoresIndex += 1
                if last100ScoresIndex >= 100:
                    last100Filled = True
                    last100ScoresIndex = 0
                if not last100Filled:
                    print("EP "+str(epoch)+" - {} timesteps".format(t+1)+"   Exploration="+str(round(explorationRate, 2)))
                else:
                    m, s = divmod(int(time.time() - start_time), 60)
                    h, m = divmod(m, 60)
                    print("EP "+str(epoch)+" - {} timesteps".format(t+1)+" - last100 Steps : "+str((sum(last100Scores)/len(last100Scores)))+" - Cumulated R: "+str(cumulated_reward)+"   Eps="+str(round(explorationRate, 2))+"     Time: %d:%02d:%02d" % (h, m, s))
                    if epoch % 100 == 0:
                        # save model weights and monitoring data every 100 epochs.
                        deepQ.saveModel(model_output+'circle2_fcnn_ep'+str(epoch)+'.h5')
                        # env.monitor.flush()
                        # copy_tree(outdir, model_output+'circle2_fcnn_ep'+str(epoch))
                        # save simulation parameters.
                        parameter_keys = ['epochs', 'steps', 'updateTargetNetwork', 'explorationRate', 'minibatch_size', 'learnStart', 'learningRate', 'discountFactor', 'memorySize', 'network_inputs', 'network_outputs', 'network_structure', 'current_epoch']
                        parameter_values = [epochs, steps, updateTargetNetwork, explorationRate, minibatch_size, learnStart, learningRate, discountFactor, memorySize, network_inputs, network_outputs, network_structure, epoch]
                        parameter_dictionary = dict(zip(parameter_keys, parameter_values))
                        with open(model_output+'circle2_fcnn_ep'+str(epoch)+'.json', 'w') as outfile:
                            json.dump(parameter_dictionary, outfile)
                break

            stepCounter += 1
            if stepCounter % updateTargetNetwork == 0:
                deepQ.updateTargetNetwork()
                print("updating target network")

        list_rewards.append(cumulated_reward)
        if(epoch%20==0):
            plotter.plot(list_rewards)
        if(epoch%100==0):
            plotter.save(outdir, epoch)

        explorationRate *= 0.998  # epsilon decay
        # explorationRate -= (2.0/epochs)
        explorationRate = max(0.05, explorationRate)

    # env.monitor.close()
    env.close()
